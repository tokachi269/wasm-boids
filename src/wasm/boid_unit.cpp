// Boid 個体レベルの物理・行動計算エンジン
// 群れ行動（結合・分離・整列）、境界回避、近傍管理を実装する中核モジュール。
// ユニフォームグリッド空間インデックスと連携し、リアルタイム群集シミュレーションを実現。

#define GLM_ENABLE_EXPERIMENTAL

#include "boid_unit.h"

#include "boids_buffers.h"
#include "dbvh_index.h"
#include "pool_accessor.h"
#include "simulation_tuning.h"
#include "species_params.h"
#include "uniform_grid.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>

extern std::vector<SpeciesParams> globalSpeciesParams;

namespace {
struct HybridNeighborScratch {
  std::vector<std::pair<float, int>> combined;
  std::vector<int> fallback;
};

inline HybridNeighborScratch &neighborScratch() {
  thread_local HybridNeighborScratch scratch;
  if (scratch.combined.capacity() < 512) {
    scratch.combined.reserve(512);
  }
  if (scratch.fallback.capacity() < 512) {
    scratch.fallback.reserve(512);
  }
  return scratch;
}
} // namespace

GridDbvhNeighborProvider::GridDbvhNeighborProvider(
    const SoABuffers &buffers_, const UniformGridIndex &gridIndex,
    const DbvhIndex *dbvhIndex, int fallbackThreshold, int fallbackLimit,
    float fallbackRadiusScale, HybridNeighborStats *statsRef)
    : buffers(buffers_), grid(gridIndex), dbvh(dbvhIndex),
      threshold(std::max(fallbackThreshold, 0)),
      fallbackCapacity(std::max(fallbackLimit, 0)),
      radiusScale(std::max(fallbackRadiusScale, 1.0f)), stats(statsRef) {}

int GridDbvhNeighborProvider::gatherNearest(const glm::vec3 &center,
                                            int maxCount, float maxRadius,
                                            int *outIndices,
                                            float *outDistancesSq) const {
  const int gathered =
      grid.gatherNearest(center, maxCount, maxRadius, outIndices,
                         outDistancesSq);

  if (stats) {
    stats->queryCount++;
    if (threshold > 0 && gathered >= threshold) {
      stats->overflowCount++;
    }
  }

  if (!dbvh || threshold <= 0 || fallbackCapacity <= 0 || maxRadius <= 0.0f ||
      gathered <= threshold) {
    return gathered;
  }

  if (stats) {
    stats->fallbackQueries++;
  }

  HybridNeighborScratch &scratch = neighborScratch();
  auto &combined = scratch.combined;
  combined.clear();
  combined.reserve(static_cast<std::size_t>(gathered));
  for (int i = 0; i < gathered; ++i) {
    combined.emplace_back(outDistancesSq[i], outIndices[i]);
  }

  auto &fallback = scratch.fallback;
  const std::size_t desiredSize = static_cast<std::size_t>(fallbackCapacity);
  if (fallback.size() < desiredSize) {
    fallback.resize(desiredSize);
  }

  const float queryRadius = maxRadius * radiusScale;
  const int fetched = dbvh->gatherWithinRadius(center, queryRadius,
                                               fallback.data(),
                                               fallbackCapacity);
  if (fetched <= 0) {
    return gathered;
  }

  const float radiusSq = maxRadius * maxRadius;
  const auto &positions = buffers.positions;
  const std::size_t boidCount = positions.size();

  auto isDuplicate = [&combined](int candidate) {
    for (const auto &entry : combined) {
      if (entry.second == candidate) {
        return true;
      }
    }
    return false;
  };

  for (int i = 0; i < fetched; ++i) {
    const int candidate = fallback[static_cast<std::size_t>(i)];
    if (candidate < 0 || candidate >= static_cast<int>(boidCount)) {
      continue;
    }
    if (isDuplicate(candidate)) {
      continue;
    }
    const glm::vec3 diff =
        positions[static_cast<std::size_t>(candidate)] - center;
    const float distSq = glm::dot(diff, diff);
    if (distSq > radiusSq) {
      continue;
    }
    combined.emplace_back(distSq, candidate);
  }

  if (combined.empty()) {
    return 0;
  }

  std::sort(combined.begin(), combined.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  const int finalCount =
      std::min(static_cast<int>(combined.size()), maxCount);
  if (stats) {
    const int added = std::max(0, finalCount - gathered);
    stats->fallbackAdded += added;
  }
  for (int i = 0; i < finalCount; ++i) {
    outDistancesSq[i] = combined[static_cast<std::size_t>(i)].first;
    outIndices[i] = combined[static_cast<std::size_t>(i)].second;
  }
  return finalCount;
}

bool GridDbvhNeighborProvider::getLeafMembers(int boidIndex,
                                              const int *&outIndices,
                                              int &count) const {
  return grid.getLeafMembers(boidIndex, outIndices, count);
}

namespace {

// 計算用の小さな定数群：浮動小数点演算の安定性と近傍管理の制御パラメータ
constexpr float kEpsilon = 1e-6f;     // ゼロ除算防止用の微小値
constexpr float kMinDirLen2 = 1e-12f; // 方向ベクトル正規化の最小二乗長
constexpr int kMaxNeighborSlots = 16; // 近傍キャッシュの最大スロット数
constexpr float kNeighborDropHysteresis = 1.15f; // 近傍除外時のヒステリシス係数

// 近傍スロット管理用ビットマスク操作：高効率な存在フラグ管理
// 16個の近傍スロットの使用状況を単一の uint16_t で管理する仕組み
inline bool maskTest(uint16_t mask, int slot) {
  return ((mask >> slot) & 1u) != 0;
}

inline void maskSet(uint16_t &mask, int slot) {
  mask |= static_cast<uint16_t>(1u << slot);
}

inline void maskReset(uint16_t &mask, int slot) {
  mask &= static_cast<uint16_t>(~(1u << slot));
}

// 軸周りの近似回転（小角度向け高速化）
// 三角関数を使わずに外積で近似計算し、リアルタイム性能を優先
inline glm::vec3 approxRotate(const glm::vec3 &v, const glm::vec3 &axis,
                              float angle) {
  const float axisLen2 = glm::length2(axis);
  if (axisLen2 < 1e-8f) {
    return v;
  }
  const glm::vec3 normalizedAxis = axis * (1.0f / glm::sqrt(axisLen2));
  return v + angle * glm::cross(normalizedAxis, v);
}

// 方向ベクトルからロールゼロのクォータニオンを生成
// Boid の姿勢表現で、前方向は指定、ロール（Z軸回転）は無しの標準姿勢を作る
inline glm::quat dirToQuatRollZero(const glm::vec3 &forward) {
  glm::vec3 f = glm::normalize(forward);
  glm::vec3 up(0.0f, 1.0f, 0.0f);
  if (std::fabs(glm::dot(f, up)) > 0.99f) {
    up = glm::vec3(1.0f, 0.0f, 0.0f);
  }
  glm::vec3 right = glm::normalize(glm::cross(up, f));
  up = glm::cross(f, right);
  glm::mat3 basis(right, up, f);
  return glm::quat_cast(basis);
}

// イージング関数群：滑らかな値変化のためのカーブ計算
// Boid の行動変化や境界反応で急激な変化を避けるために使用
inline float easeOut(float t) { return t * t * (3.0f - 2.0f * t); }

inline float smoothStep(float edge0, float edge1, float x) {
  if (edge0 == edge1) {
    return x < edge0 ? 0.0f : 1.0f;
  }
  const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// ゼロベクトルを安全に扱う正規化：長さがゼロに近い場合の例外処理
// Boid の速度や力ベクトルの計算で数値的不安定性を回避
inline glm::vec3 safeNormalize(const glm::vec3 &v) {
  const float lenSq = glm::dot(v, v);
  if (lenSq < kMinDirLen2) {
    return glm::vec3(0.0f);
  }
  return v * glm::inversesqrt(lenSq);
}

// 軽量な乱数生成器（スレッドローカル）
// 線形合同法による高速擬似乱数、マルチスレッド環境で独立したシード管理
inline uint32_t nextRandom() {
  static thread_local uint32_t state = 1u;
  state = state * 1103515245u + 12345u;
  return (state >> 16) & 0x7fffu;
}

// 近傍キャッシュの空きスロット検索：ビットマスクを線形走査
// 新しい近傍を追加する際の高速な空き領域発見
inline int findFreeSlot(uint16_t mask, int limit) {
  for (int i = 0; i < limit; ++i) {
    if (!maskTest(mask, i)) {
      return i;
    }
  }
  return -1;
}

// 重複近傍の検出：同一 Boid を複数回近傍登録することを防ぐ
// キャッシュ整合性維持のための事前チェック機能
inline bool containsNeighbor(uint16_t mask,
                             const std::array<int, kMaxNeighborSlots> &slots,
                             int limit, int candidate) {
  for (int i = 0; i < limit; ++i) {
    if (maskTest(mask, i) && slots[i] == candidate) {
      return true;
    }
  }
  return false;
}

// 全種族パラメータから最大の捕食者警戒半径を算出
// グリッド近傍探索の半径パラメータ決定に使用、過不足ない探索範囲を保証
float computeMaxPredatorAlertRadius() {
  float radius = 1.0f;
  for (const auto &params : globalSpeciesParams) {
    if (params.isPredator) {
      continue;
    }
    radius = std::max(radius, std::max(params.predatorAlertRadius, 0.0f));
  }
  return radius;
}

} // namespace

namespace {

// Boid 相互作用計算の中核関数：指定範囲の個体について群れ行動力を算出
// 近傍探索・キャッシュ管理・三大法則（結合・分離・整列）・捕食回避を統合実行
void computeBoidInteractionsSpan(SoABuffers &buf,
                                 const GridDbvhNeighborProvider &neighbors,
                                 int begin, int end, float dt,
                                 float maxPredatorAlertRadius) {
  const int totalCount = static_cast<int>(buf.positions.size());
  if (totalCount == 0) {
    return;
  }

  begin = std::clamp(begin, 0, totalCount);
  end = std::clamp(end, begin, totalCount);
  if (begin >= end) {
    return;
  }

  // 近傍候補の処理プール上限（メモリ効率と性能のバランス）
  constexpr int kMaxCandidatePool = kMaxNeighborSlots * 2;
  constexpr int kMaxPredatorCandidates = 64;

  for (int gIdx = begin; gIdx < end; ++gIdx) {
    const int sid = buf.speciesIds[gIdx];
    if (sid < 0 || sid >= static_cast<int>(globalSpeciesParams.size())) {
      continue;
    }
    const SpeciesParams &selfParams = globalSpeciesParams[sid];

    glm::vec3 newAcceleration(0.0f);

    // 現在の Boid の近傍キャッシュと状態データへの参照取得
    auto &cohesionMemories = buf.boidCohesionMemories[gIdx];
    uint16_t &activeNeighbors = buf.boidNeighborMasks[gIdx];
    auto &neighborIndices = buf.boidNeighborIndices[gIdx];

    // 近傍スロット数の動的調整：種族パラメータに基づく最適化
    const int desiredSlots =
        std::min(kMaxNeighborSlots, std::max(1, selfParams.maxNeighbors));
    if (static_cast<int>(cohesionMemories.size()) < desiredSlots) {
      cohesionMemories.resize(desiredSlots, 0.0f);
    }
    const int slotCount = std::min(desiredSlots, kMaxNeighborSlots);

    // 未使用スロットのクリア：メモリ安全性とキャッシュ整合性の保証
    for (int slot = slotCount; slot < kMaxNeighborSlots; ++slot) {
      maskReset(activeNeighbors, slot);
      neighborIndices[slot] = -1;
    }

    // 現在 Boid の基本状態：位置・速度・向きの取得と正規化
    const glm::vec3 pos = buf.positions[gIdx];
    const glm::vec3 vel = buf.velocities[gIdx];
    const float velLen2 = glm::length2(vel);
    glm::vec3 forward = safeNormalize(vel);
    if (glm::length2(forward) < 1e-8f) {
      forward = glm::vec3(0.0f, 0.0f, 1.0f); // 静止時のデフォルト向き
    }

    // 近傍探索パラメータの算出：結合・分離半径から最適なクエリ範囲を決定
    const float queryRadius = std::max(
        selfParams.cohesionRange, std::max(selfParams.separationRange, 0.0f));
    const float viewRangeSq = (queryRadius > 0.0f)
                                  ? queryRadius * queryRadius
                                  : std::numeric_limits<float>::infinity();
    const float halfFovRad =
        glm::radians(std::max(selfParams.fieldOfViewDeg, 0.0f) * 0.5f);
    const float cosHalfFov = std::cos(halfFovRad);

    // 群れ行動の距離閾値：分離（Separation）・結合（Cohesion）の二乗距離
    const float reSq = selfParams.separationRange * selfParams.separationRange;
    const float raSq = selfParams.cohesionRange * selfParams.cohesionRange;

    // 近傍キャッシュ管理：ヒステリシスによる安定した近傍選択
    const float dropRadiusSq =
        (raSq > 0.0f ? raSq * kNeighborDropHysteresis : viewRangeSq);
    const float searchRadiusLimit =
        (raSq > 0.0f) ? glm::sqrt(dropRadiusSq) : queryRadius;

    // 既存近傍キャッシュの検証・更新：距離・時間・種族による無効化判定
    int activeCount = 0;
    for (int slot = 0; slot < slotCount; ++slot) {
      if (!maskTest(activeNeighbors, slot)) {
        neighborIndices[slot] = -1;
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
        continue;
      }

      // 基本的な有効性チェック：インデックス範囲・種族一致
      const int neighborIdx = neighborIndices[slot];
      bool remove = neighborIdx < 0 || neighborIdx >= totalCount;
      if (!remove && buf.speciesIds[neighborIdx] != sid) {
        remove = true; // 異種族は近傍から除外
      }

      // 距離ベース除外：ヒステリシス付きで遠すぎる近傍を削除
      if (!remove) {
        const glm::vec3 diff = buf.positions[neighborIdx] - pos;
        const float distSq = glm::dot(diff, diff);
        if (distSq > dropRadiusSq) {
          remove = true;
        }
      }

      // 時間ベース除外：長期間維持された近傍の自動リフレッシュ（tau 時定数）
      if (!remove && slot < static_cast<int>(cohesionMemories.size())) {
        cohesionMemories[slot] += dt;
        if (cohesionMemories[slot] > selfParams.tau) {
          remove = true; // 時間経過による近傍の陳腐化防止
        }
      }

      // 近傍スロットの更新：無効な近傍を削除または有効カウントを増加
      if (remove) {
        maskReset(activeNeighbors, slot);
        neighborIndices[slot] = -1;
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
      } else {
        ++activeCount;
      }
    }

    const int neededNeighbors = std::max(0, slotCount - activeCount);
    const bool needsNeighborRefresh = neededNeighbors > 0;
    std::array<float, kMaxCandidatePool> candidateDistSquared{};
    std::array<int, kMaxCandidatePool> candidateIndices{};
    int candidateCount = 0;

    std::array<int, kMaxPredatorCandidates> predatorCandidates{};
    int predatorCandidateCount = 0;

    glm::vec3 attractDirSum(0.0f);
    int attractDirCount = 0;

    // -----------------------------------------------
    // 近傍探索の準備: 視界パラメータと探索範囲を設定
    // -----------------------------------------------
    // queryRadius: cohesionRange と separationRange
    // の大きい方を最大探索半径とする。
    // 実際の探索では小さい半径から段階的に拡大することで密集時の無駄を削減。
    int candidateCapacity = 0;
    if (needsNeighborRefresh && searchRadiusLimit > 0.0f) {
      // -----------------------------------------------
      // 候補プールのキャパシティ計算
      // -----------------------------------------------
      // 必要スロット数 + 予備枠を確保して、視界内の有効な候補を絞り込む。
      const int baseCapacity = std::max(neededNeighbors, 1);
      const int paddedCapacity =
          std::max(baseCapacity, baseCapacity + neededNeighbors / 2);
      candidateCapacity = std::max(
          1, std::min(kMaxCandidatePool, std::max(paddedCapacity, slotCount)));

      // -----------------------------------------------
      // まず同一葉ノード内から候補を収集し、不足分のみ広域探索する
      // -----------------------------------------------
      const int stopThreshold = std::max(neededNeighbors, 1);
      std::array<std::pair<float, int>, kMaxCandidatePool> localPairs{};
      int localPairCount = 0;
      const int *leafMembers = nullptr;
      int leafCount = 0;
  if (neighbors.getLeafMembers(gIdx, leafMembers, leafCount)) {
        for (int i = 0; i < leafCount && localPairCount < candidateCapacity;
             ++i) {
          const int neighborIdx = leafMembers[i];
          if (neighborIdx == gIdx || neighborIdx < 0 ||
              neighborIdx >= totalCount) {
            continue;
          }
          if (buf.speciesIds[neighborIdx] != sid) {
            continue;
          }
          if (containsNeighbor(activeNeighbors, neighborIndices, slotCount,
                               neighborIdx)) {
            continue;
          }
          const glm::vec3 diff = buf.positions[neighborIdx] - pos;
          const float distSq = glm::dot(diff, diff);
          if (distSq < kEpsilon || distSq >= dropRadiusSq) {
            continue;
          }
          const float diffDot = glm::dot(forward, diff);
          const float requiredDot = cosHalfFov * glm::sqrt(distSq);
          if (diffDot < requiredDot) {
            continue;
          }

          bool duplicate = false;
          for (int j = 0; j < localPairCount; ++j) {
            if (localPairs[j].second == neighborIdx) {
              duplicate = true;
              break;
            }
          }
          if (duplicate) {
            continue;
          }
          localPairs[localPairCount++] = std::make_pair(distSq, neighborIdx);
        }

        if (localPairCount > 1) {
          std::sort(
              localPairs.begin(), localPairs.begin() + localPairCount,
              [](const auto &a, const auto &b) { return a.first < b.first; });
        }

        for (int i = 0;
             i < localPairCount && candidateCount < candidateCapacity; ++i) {
          candidateDistSquared[candidateCount] = localPairs[i].first;
          candidateIndices[candidateCount] = localPairs[i].second;
          ++candidateCount;
          if (candidateCount >= stopThreshold) {
            break;
          }
        }
      }

      // -----------------------------------------------
      // 不足している場合のみ段階的半径拡張でグリッド検索を実行
      // -----------------------------------------------
      if (candidateCount < stopThreshold) {
        const float radiusLimit = searchRadiusLimit;
        float searchRadius = std::max(selfParams.separationRange, 0.0f);
        if (searchRadius <= 0.0f) {
          searchRadius = radiusLimit * 0.25f;
        }
        const float minRadius = std::min(0.5f, radiusLimit);
        searchRadius = std::clamp(searchRadius, minRadius, radiusLimit);

        std::array<int, kMaxCandidatePool> queryIndices{};
        std::array<float, kMaxCandidatePool> queryDistSq{};
        int passes = 0;
        while (candidateCount < candidateCapacity && passes < 6) {
          const int remaining = stopThreshold - candidateCount;
          if (remaining <= 0) {
            break;
          }
          const int maxRoom = candidateCapacity - candidateCount;
          const int requestCount = std::max(1, std::min(remaining, maxRoom));

      const int fetched = neighbors.gatherNearest(
        pos, requestCount, searchRadius, queryIndices.data(),
        queryDistSq.data());

          for (int i = 0; i < fetched && candidateCount < candidateCapacity;
               ++i) {
            const int neighborIdx = queryIndices[i];
            const float distSq = queryDistSq[i];
            if (neighborIdx == gIdx || neighborIdx < 0 ||
                neighborIdx >= totalCount) {
              continue;
            }
            if (buf.speciesIds[neighborIdx] != sid) {
              continue;
            }
            if (distSq < kEpsilon || distSq >= dropRadiusSq) {
              continue;
            }

            const glm::vec3 diff = buf.positions[neighborIdx] - pos;
            const float diffDot = glm::dot(forward, diff);
            const float requiredDot = cosHalfFov * glm::sqrt(distSq);
            if (diffDot < requiredDot) {
              continue;
            }
            if (containsNeighbor(activeNeighbors, neighborIndices, slotCount,
                                 neighborIdx)) {
              continue;
            }
            bool duplicate = false;
            for (int j = 0; j < candidateCount; ++j) {
              if (candidateIndices[j] == neighborIdx) {
                duplicate = true;
                break;
              }
            }
            if (duplicate) {
              continue;
            }

            candidateDistSquared[candidateCount] = distSq;
            candidateIndices[candidateCount] = neighborIdx;
            ++candidateCount;
            if (candidateCount >= stopThreshold) {
              break;
            }
          }

          if (candidateCount >= stopThreshold || searchRadius >= radiusLimit) {
            break;
          }

          float nextRadius = searchRadius * 1.5f;
          if (nextRadius <= searchRadius + 1e-3f) {
            nextRadius = searchRadius + 1.0f;
          }
          searchRadius = std::min(radiusLimit, nextRadius);
          ++passes;
        }
      }

      if (candidateCount > 1) {
        std::array<std::pair<float, int>, kMaxCandidatePool> sortedPairs{};
        for (int i = 0; i < candidateCount; ++i) {
          sortedPairs[i] =
              std::make_pair(candidateDistSquared[i], candidateIndices[i]);
        }
        std::sort(
            sortedPairs.begin(), sortedPairs.begin() + candidateCount,
            [](const auto &a, const auto &b) { return a.first < b.first; });
        for (int i = 0; i < candidateCount; ++i) {
          candidateDistSquared[i] = sortedPairs[i].first;
          candidateIndices[i] = sortedPairs[i].second;
        }
      }
    }

    // -----------------------------------------------
    // 近傍スロットへの候補追加
    // -----------------------------------------------
    const int toAddTarget = std::max(0, desiredSlots - activeCount);
    if (needsNeighborRefresh && toAddTarget > 0 && candidateCount > 0) {
      // 追加可能な候補数を制限し、近いものから順に登録
      const int limitedAdd = std::min(toAddTarget, candidateCount);
      for (int i = 0; i < limitedAdd; ++i) {
        const int neighborIdx = candidateIndices[i];
        // 重複チェック（理論上はフィルタ済みだが念のため）
        if (containsNeighbor(activeNeighbors, neighborIndices, slotCount,
                             neighborIdx)) {
          continue;
        }
        const int slot = findFreeSlot(activeNeighbors, slotCount);
        if (slot < 0) {
          break;
        }
        maskSet(activeNeighbors, slot);
        neighborIndices[slot] = neighborIdx;
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = dt;
        }
        ++activeCount;
      }
    }

    // -----------------------------------------------
    // 捕食者処理: 獲物の探知とターゲット選択
    // -----------------------------------------------
    if (selfParams.isPredator) {
      int &targetIndex = buf.predatorTargetIndices[gIdx];
      float &targetTime = buf.predatorTargetTimers[gIdx];
      targetTime -= dt;

      std::array<float, kMaxPredatorCandidates> predatorDists{};
      // 最大警戒範囲内の獲物候補をグリッドから取得
    const int predatorFetched = neighbors.gatherNearest(
      pos, kMaxPredatorCandidates, maxPredatorAlertRadius,
      predatorCandidates.data(), predatorDists.data());

      // -----------------------------------------------
      // 獲物候補のフィルタリングと恐怖影響の付与
      // -----------------------------------------------
      for (int i = 0; i < predatorFetched; ++i) {
        const int preyIdx = predatorCandidates[i];
        // 自身・範囲外・無効インデックスを除外
        if (preyIdx == gIdx || preyIdx < 0 || preyIdx >= totalCount) {
          continue;
        }
        const int preySid = buf.speciesIds[preyIdx];
        if (preySid < 0 ||
            preySid >= static_cast<int>(globalSpeciesParams.size())) {
          continue;
        }
        // 捕食者同士は対象外
        if (globalSpeciesParams[preySid].isPredator) {
          continue;
        }

        const SpeciesParams &preyParams = globalSpeciesParams[preySid];
        // 獲物の警戒範囲を取得（未設定時は最大値を使用）
        float alertRadius = std::max(preyParams.predatorAlertRadius, 0.0f);
        if (alertRadius <= 0.0f) {
          alertRadius = maxPredatorAlertRadius;
        }
        const float alertRadiusSq = alertRadius * alertRadius;
        const float distSq = predatorDists[i];
        if (distSq >= alertRadiusSq || distSq < kEpsilon) {
          continue;
        }

        // 距離に応じた逃避強度を計算（smoothstep 補間）
        const float dist = glm::sqrt(std::max(distSq, kEpsilon));
        float escapeStrength =
            std::clamp(1.0f - dist / alertRadius, 0.0f, 1.0f);
        escapeStrength =
            escapeStrength * escapeStrength * (3.0f - 2.0f * escapeStrength);

        // 獲物に捕食者からの逃避影響を付与
        const glm::vec3 diff = buf.positions[preyIdx] - pos;
        const glm::vec3 escapeDir = diff / dist;
        buf.predatorInfluences[preyIdx] += escapeDir * escapeStrength * 5.0f;
        buf.predatorThreats[preyIdx] =
            std::max(buf.predatorThreats[preyIdx], escapeStrength);
        buf.stresses[preyIdx] =
            std::max(buf.stresses[preyIdx],
                     std::clamp(0.4f + escapeStrength * 0.6f, 0.0f, 1.0f));

        // 有効な獲物候補リストに追加
        if (predatorCandidateCount < kMaxPredatorCandidates) {
          predatorCandidates[predatorCandidateCount++] = preyIdx;
        }
      }

      // -----------------------------------------------
      // ターゲット検証と再選択
      // -----------------------------------------------
      const bool targetInvalid =
          (targetIndex < 0) || (targetTime <= 0.0f) ||
          buf.speciesIds[targetIndex] < 0 ||
          buf.speciesIds[targetIndex] >=
              static_cast<int>(globalSpeciesParams.size()) ||
          globalSpeciesParams[buf.speciesIds[targetIndex]].isPredator;
      if (targetInvalid) {
        targetIndex = -1;
        if (predatorCandidateCount > 0) {
          const int pick =
              predatorCandidates[nextRandom() % predatorCandidateCount];
          targetIndex = pick;
          targetTime = selfParams.tau;
        } else {
          targetTime = 0.0f;
        }
      }

      // ターゲットへの追跡加速
      if (targetIndex >= 0 && targetIndex < totalCount) {
        const glm::vec3 diff = buf.positions[targetIndex] - pos;
        const float distSq = glm::dot(diff, diff);
        if (distSq > kEpsilon) {
          const glm::vec3 chaseDir = diff * (1.0f / glm::sqrt(distSq));
          const glm::vec3 desiredVel = chaseDir * selfParams.maxSpeed;
          newAcceleration += desiredVel - vel;
        }
      }
    }

    // 誘引状態の管理（孤立時に仲間の方向へ向かう）
    const float neighborRatio =
        activeCount > 0
            ? static_cast<float>(activeCount) /
                  static_cast<float>(std::max(1, selfParams.maxNeighbors))
            : 0.0f;
    if (neighborRatio < 1.0f) {
      buf.isAttracting[gIdx] = 1;
      buf.attractTimers[gIdx] = selfParams.tau;
    } else if (buf.isAttracting[gIdx]) {
      // 近傍が充足した後もタイマーが切れるまで誘引を継続
      buf.attractTimers[gIdx] -= dt;
      if (buf.attractTimers[gIdx] <= 0.0f) {
        buf.isAttracting[gIdx] = 0;
        buf.attractTimers[gIdx] = 0.0f;
      }
    }

    // -----------------------------------------------
    // 誘引加速度の適用（孤立時に仲間の方向へ向かう）
    // -----------------------------------------------
    if (buf.isAttracting[gIdx]) {
      if (attractDirCount > 0) {
        const glm::vec3 avgDir =
            safeNormalize(attractDirSum / static_cast<float>(attractDirCount));
        const glm::vec3 desiredVel = avgDir * selfParams.maxSpeed;
        const glm::vec3 attractAcc = (desiredVel - vel) * selfParams.lambda;
        newAcceleration += attractAcc;
      }
    }

    // 近傍が存在しない場合は誘引加速度のみで終了
    if (activeCount == 0) {
      buf.accelerations[gIdx] = newAcceleration;
      continue;
    }

    // ★ Boids 三大法則の力ベクトル累積変数
    // Separation（分離）: 近すぎる個体から離れる力
    // Alignment（整列）: 周囲の個体と速度を合わせる力
    // Cohesion（結合）: 群れの中心に向かう力
    glm::vec3 sumSeparation(0.0f);
    glm::vec3 sumAlignment(0.0f);
    glm::vec3 sumCohesion(0.0f);
    float stressGainSum = 0.0f; // ストレス値の伝播計算用
    float stressWeightSum = 0.0f;

    const float selfStress = buf.stresses[gIdx];
    const float threatLevel = std::clamp(buf.predatorThreats[gIdx], 0.0f, 1.0f);
    const float propagationRadius = std::max(selfParams.cohesionRange, 1.0f);

    // -----------------------------------------------
    // 近傍スロットループ: 各近傍との相互作用を計算
    // -----------------------------------------------
    for (int slot = 0; slot < slotCount; ++slot) {
      // スロットが未使用の場合はスキップ
      if (!maskTest(activeNeighbors, slot)) {
        continue;
      }
      const int neighborIdx = neighborIndices[slot];
      // 無効インデックスはスロットをクリア
      if (neighborIdx < 0 || neighborIdx >= totalCount) {
        maskReset(activeNeighbors, slot);
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
        continue;
      }

      // 近傍の種族チェック（無効種族はスロットをクリア）
      const int neighborSid = buf.speciesIds[neighborIdx];
      if (neighborSid < 0 ||
          neighborSid >= static_cast<int>(globalSpeciesParams.size())) {
        maskReset(activeNeighbors, slot);
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
        continue;
      }
      const SpeciesParams &neighborParams = globalSpeciesParams[neighborSid];

      // 近傍との空間関係を計算：相対位置ベクトルと距離
      const glm::vec3 diff = buf.positions[neighborIdx] - pos;
      const float distSq = glm::dot(diff, diff);
      if (distSq <= kEpsilon) {
        continue; // 同一位置にある場合は処理をスキップ
      }
      const float dist = glm::sqrt(distSq);

      // ★ 物理的衝突回避：体の半径による硬い反発力
      // 個体同士が重なり合わないよう、距離に反比例した強力な押し戻し力を適用
      const float selfRadius = std::max(selfParams.bodyRadius, 0.0f);
      const float neighborRadius = std::max(neighborParams.bodyRadius, 0.0f);
      const float combinedRadius = selfRadius + neighborRadius;
      if (combinedRadius > 1e-5f) {
        const float clearanceSq = combinedRadius * combinedRadius;
        if (distSq < clearanceSq) {
          const float distSafe = glm::sqrt(std::max(distSq, kEpsilon));
          const float penetration =
              clearanceSq > 0.0f ? std::max(combinedRadius - distSafe, 0.0f)
                                 : 0.0f;
          if (penetration > 0.0f) {
            // 貫通の深さに比例した二乗カーブで反発力を強化
            const glm::vec3 normal = diff * (1.0f / distSafe);
            const float penetrationRatio =
                penetration / std::max(combinedRadius, 1e-5f);
            const float response = penetrationRatio * penetrationRatio * 10.0f;
            const float selfImpulse = response *
                                      std::max(selfParams.separation, 0.02f) *
                                      (1.0f + selfParams.maxSpeed);
            sumSeparation += normal * selfImpulse;
          }
        }
      }

      // -----------------------------------------------
      // ストレス伝播: 近くの高ストレス個体から影響を受ける
      // -----------------------------------------------
      if (dist < propagationRadius) {
        const float neighborStress = buf.stresses[neighborIdx];
        // 自分より高ストレスの近傍からストレスを受け取る
        if (neighborStress > selfStress) {
          const float stressStrength = smoothStep(0.25f, 0.75f, neighborStress);
          const float distanceFactor =
              std::clamp(1.0f - dist / propagationRadius, 0.0f, 1.0f);
          if (distanceFactor > 0.0f) {
            const float weight = stressStrength * distanceFactor;
            stressGainSum += neighborStress * weight;
            stressWeightSum += weight;
          }
        }
      }

      // ★ 分離力（Separation）：近すぎる個体から離れる反発力
      // 距離に反比例する力で、密集を防ぎながら個体間のパーソナルスペースを維持
      float separationRange = selfParams.separationRange;
      if (separationRange <= 1e-4f) {
        // 分離範囲が未設定の場合は体サイズから自動推定
        const float bodyDiameter = std::max(selfParams.bodyRadius * 2.0f, 0.0f);
        const float bodyLength = std::abs(selfParams.bodyHeadLength) +
                                 std::abs(selfParams.bodyTailLength);
        separationRange = std::max(bodyDiameter, bodyLength);
      }
      float wSep = 0.0f;
      if (separationRange > 1e-4f) {
        wSep = std::clamp(1.0f - (dist / separationRange), 0.0f, 1.0f);
      }
      // 反対方向（-diff）に距離の逆数に比例した力を加える
      sumSeparation += (-diff) * (wSep / std::max(distSq, kEpsilon));

      // ★ 結合指向の蓄積：分離範囲外で結合範囲内の個体への誘引
      // 適度な距離の近傍に向かう方向を記録（後で結合力計算に使用）
      if (distSq > reSq && distSq <= raSq) {
        attractDirSum += diff * (1.0f / dist);
        ++attractDirCount;
      }

      // ★ 結合力（Cohesion）：群れの中心に向かう求心力
      // 近傍の加重平均位置を計算し、そこに向かう力を生成
      float wCohesion = std::clamp(
          dist / std::max(selfParams.cohesionRange, 1e-4f), 0.0f, 1.0f);
      const float stressFactor = 1.0f + buf.stresses[gIdx] * 0.2f;
      const float cohesionThreatFactor =
          1.0f + gSimulationTuning.cohesionBoost * threatLevel;
      wCohesion *= stressFactor * cohesionThreatFactor;
      sumCohesion += buf.positions[neighborIdx] * wCohesion;

      // ★ 整列力（Alignment）：周囲と速度を合わせる協調行動
      // 近傍の速度ベクトルを累積して平均化し、統一された移動方向を形成
      sumAlignment += buf.velocities[neighborIdx];
    }

    // ストレス伝播の適用
    if (stressWeightSum > 0.0f) {
      const float propagatedStress = stressGainSum / stressWeightSum;
      const float delta = propagatedStress - selfStress;
      if (delta > 0.0f) {
        const float blend =
            std::clamp(gSimulationTuning.threatGain * dt, 0.0f, 0.6f);
        buf.stresses[gIdx] = std::clamp(selfStress + delta * blend, 0.0f, 1.0f);
      }
    }

    // 分離力の正規化と脅威係数の適用
    glm::vec3 totalSeparation(0.0f);
    const float sepLen2 = glm::length2(sumSeparation);
    if (sepLen2 > kMinDirLen2) {
      const float separationThreatFactor =
          std::lerp(1.0f, gSimulationTuning.separationMinFactor, threatLevel);
      totalSeparation = safeNormalize(sumSeparation) *
                        (selfParams.separation * separationThreatFactor);
    }

    // -----------------------------------------------
    // 凝集力の正規化
    // -----------------------------------------------
    glm::vec3 totalCohesion(0.0f);
    if (activeCount > 0) {
      const glm::vec3 avgCohesionPos =
          sumCohesion * (1.0f / static_cast<float>(activeCount));
      glm::vec3 cohesionDir = avgCohesionPos - pos;
      const glm::vec3 cohesionNorm = safeNormalize(cohesionDir);
      if (glm::length2(cohesionNorm) > kMinDirLen2) {
        totalCohesion = cohesionNorm * selfParams.cohesion;
      }
    }

    // 整列力の正規化と脅威係数の適用
    glm::vec3 totalAlignment(0.0f);
    glm::vec3 torqueImpulse(0.0f);
    if (activeCount > 0) {
      glm::vec3 avgAlignVel =
          sumAlignment * (1.0f / static_cast<float>(activeCount));
      glm::vec3 aliDir = avgAlignVel - vel;
      const float aliLen2 = glm::length2(aliDir);
      if (aliLen2 > kMinDirLen2) {
        const float alignmentThreatFactor =
            1.0f + gSimulationTuning.alignmentBoost * threatLevel;
        totalAlignment = safeNormalize(aliDir) *
                         (selfParams.alignment * alignmentThreatFactor);
      }
    }

    // -----------------------------------------------
    // トルク制限: 急激な方向転換を抑制
    // -----------------------------------------------
    if (velLen2 > kMinDirLen2) {
      const float aliLen2 = glm::length2(totalAlignment);
      if (aliLen2 > kMinDirLen2) {
        const float velLen = glm::sqrt(velLen2);
        const glm::vec3 forward2 = vel * (1.0f / velLen);
        glm::vec3 tgt2 = totalAlignment * (1.0f / glm::sqrt(aliLen2));
        float dot2 = std::clamp(glm::dot(forward2, tgt2), -1.0f, 1.0f);
        float ang2 = std::acos(dot2);
        if (ang2 > 1e-4f) {
          glm::vec3 axis2 = glm::cross(forward2, tgt2);
          const float axisLen2 = glm::length2(axis2);
          if (axisLen2 > 1e-8f) {
            axis2 *= (1.0f / glm::sqrt(axisLen2));
            // 最大回転角を適用してトルクを制限
            const float rot2 = std::min(ang2, selfParams.torqueStrength * dt);
            const float rotRatio = rot2 / std::max(ang2, 1e-4f);
            totalAlignment *= rotRatio;
            torqueImpulse += axis2 * (ang2 * selfParams.torqueStrength);
          }
        }
      }
    }

    // -----------------------------------------------
    // 最終加速度の合成と適用
    // -----------------------------------------------
    newAcceleration +=
        totalSeparation + totalAlignment + totalCohesion + torqueImpulse;

    buf.accelerations[gIdx] = newAcceleration;
  }
}

} // namespace

// Boid 相互作用計算のメインエントリーポイント：指定範囲での群れ行動力算出
// ユニフォームグリッド空間インデックスを活用して効率的な近傍探索を実行し、
// 三大法則（分離・整列・結合）+ 捕食回避 + 境界回避の合成力を計算
void computeBoidInteractionsRange(SoABuffers &buf,
                                  const GridDbvhNeighborProvider &neighbors,
                                  int begin,
                                  int end, float dt) {
  const int totalCount = static_cast<int>(buf.positions.size());
  if (totalCount == 0) {
    return;
  }

  const int clampedBegin = std::clamp(begin, 0, totalCount);
  const int clampedEnd = std::clamp(end, clampedBegin, totalCount);
  if (clampedBegin >= clampedEnd) {
    return;
  }

  const int workItemCount = clampedEnd - clampedBegin;
  const float maxPredatorAlertRadius = computeMaxPredatorAlertRadius();

  constexpr int kMinChunk = 64;
  const unsigned hwConcurrency =
      std::max(1u, std::thread::hardware_concurrency());
  const bool useParallel = hwConcurrency > 1 && workItemCount >= kMinChunk;

  if (!useParallel) {
  computeBoidInteractionsSpan(buf, neighbors, clampedBegin, clampedEnd, dt,
                maxPredatorAlertRadius);
    return;
  }

  const unsigned desiredTasks = std::min<unsigned>(
      hwConcurrency,
      static_cast<unsigned>((workItemCount + kMinChunk - 1) / kMinChunk));
  const int chunkSize =
      std::max(1, (workItemCount + static_cast<int>(desiredTasks) - 1) /
                      static_cast<int>(desiredTasks));

  auto &pool = getThreadPool();
  std::vector<std::future<void>> futures;
  futures.reserve(desiredTasks > 0 ? desiredTasks - 1 : 0);

  int chunkBegin = clampedBegin;
  // 先に残りチャンクをスレッドプールへ投げ、最後のチャンクは呼び出し元スレッドで処理してオーバーヘッドを抑える。
  for (unsigned task = 1; task < desiredTasks; ++task) {
    const int chunkEnd = std::min(clampedEnd, chunkBegin + chunkSize);
    futures.emplace_back(pool.enqueue(
        [chunkBegin, chunkEnd, dt, maxPredatorAlertRadius, &buf, &neighbors]() {
          computeBoidInteractionsSpan(buf, neighbors, chunkBegin, chunkEnd, dt,
                                      maxPredatorAlertRadius);
        }));
    chunkBegin = chunkEnd;
  }

  computeBoidInteractionsSpan(buf, neighbors, chunkBegin, clampedEnd, dt,
                              maxPredatorAlertRadius);

  for (auto &fut : futures) {
    fut.get();
  }
}

// -----------------------------------------------
// Boid 運動学的更新のメインエントリーポイント：物理積分と姿勢計算
// 相互作用で計算された加速度を基に、速度・位置・姿勢を時間発展させる
// 速度制限・境界処理・姿勢のスムージングを統合実行
void updateBoidKinematicsRange(SoABuffers &buf, int begin, int end, float dt) {
  const int totalCount = static_cast<int>(buf.positions.size());
  if (totalCount == 0) {
    return;
  }
  begin = std::clamp(begin, 0, totalCount);
  end = std::clamp(end, begin, totalCount);
  if (begin >= end) {
    return;
  }

  for (int gIdx = begin; gIdx < end; ++gIdx) {
    const int sid = buf.speciesIds[gIdx];
    if (sid < 0 || sid >= static_cast<int>(globalSpeciesParams.size())) {
      continue;
    }
    const SpeciesParams &params = globalSpeciesParams[sid];

    const glm::vec3 velocity = buf.velocities[gIdx];
    const glm::vec3 acceleration = buf.accelerations[gIdx];
    const glm::vec3 position = buf.positions[gIdx];

    // -----------------------------------------------
    // ストレスに応じた速度係数の計算
    // -----------------------------------------------
    float currentStress = buf.stresses[gIdx];
    float stressFactor = 1.0f;
    if (currentStress > 0.95f) {
      const float t = (currentStress - 0.8f) / 0.2f;
      stressFactor = 1.0f + 1.6f + t * 0.9f;
    } else if (currentStress > 0.7f) {
      const float t = (currentStress - 0.2f) / 0.6f;
      const float smoothT = t * t * (3.0f - 2.0f * t);
      stressFactor = 1.0f + 0.4f + smoothT * 1.8f;
    } else {
      const float t = currentStress / 0.2f;
      stressFactor = 1.0f + t * 0.4f;
    }

    // -----------------------------------------------
    // 速度予測と方向の正規化
    // -----------------------------------------------
    glm::vec3 desiredVelocity = velocity + acceleration * dt;
    float speed = 0.0f;
    glm::vec3 oldDir = velocity;
    float oldSpeedSq = glm::length2(velocity);
    if (oldSpeedSq > kMinDirLen2) {
      oldDir /= glm::sqrt(oldSpeedSq);
    } else {
      oldDir = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    const float desiredSpeedSq = glm::length2(desiredVelocity);
    glm::vec3 newDir = oldDir;
    if (desiredSpeedSq > kMinDirLen2) {
      speed = glm::sqrt(desiredSpeedSq);
      newDir = desiredVelocity / speed;
    }

    // -----------------------------------------------
    // 最大旋回角の制限（ストレスと捕食者状態で調整）
    // -----------------------------------------------
  // SpeciesParams::maxTurnAngle は 1 ステップあたりの許容回転量を表す。
  float maxTurnAngle = params.maxTurnAngle * stressFactor;
    if (params.isPredator && buf.predatorTargetIndices[gIdx] >= 0) {
      maxTurnAngle *= 1.5f;
    }
    if (currentStress > 0.7f) {
      const float emergencyTurnFactor = 1.0f + (currentStress - 0.7f) * 5.0f;
      maxTurnAngle *= emergencyTurnFactor;
    }

    const float dotProduct = glm::dot(oldDir, newDir);
    const float angle = std::acos(std::clamp(dotProduct, -1.0f, 1.0f));
    const float allowedTurn = maxTurnAngle;
    if (angle > allowedTurn) {
      glm::vec3 axis = glm::cross(oldDir, newDir);
      const float axisLen2 = glm::length2(axis);
      if (axisLen2 > 1e-8f) {
        axis /= glm::sqrt(axisLen2);
        const float rot = std::min(angle, allowedTurn);
        newDir = approxRotate(oldDir, axis, rot);
      }
    }

    // -----------------------------------------------
    // 水平トルク: Y軸方向の傾きを補正して水平に戻す
    // -----------------------------------------------
    const float tilt = newDir.y;
    if (std::fabs(tilt) > 1e-4f) {
      glm::vec3 flatDir = glm::normalize(glm::vec3(newDir.x, 0.0f, newDir.z));
      glm::vec3 axis = glm::cross(newDir, flatDir);
      const float flatAngle =
          std::acos(std::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f));
      const float axisLen2 = glm::length2(axis);
      if (flatAngle > 1e-4f && axisLen2 > 1e-8f) {
        axis /= glm::sqrt(axisLen2);
        const float rot = std::min(flatAngle, params.horizontalTorque * dt);
        newDir = approxRotate(newDir, axis, rot);
      }
    }

    // -----------------------------------------------
    // 速度クランプと最終更新
    // -----------------------------------------------
    const float maxSpeed = params.maxSpeed * stressFactor;
    const float finalSpeed = std::clamp(speed, params.minSpeed, maxSpeed);
    const glm::vec3 newVelocity = newDir * finalSpeed;

    buf.velocitiesWrite[gIdx] = newVelocity;
    buf.positionsWrite[gIdx] = position + newVelocity * dt;
    buf.orientationsWrite[gIdx] = dirToQuatRollZero(newDir);
    buf.accelerations[gIdx] = glm::vec3(0.0f);

    // -----------------------------------------------
    // 捕食者影響と脅威の減衰
    // -----------------------------------------------
    buf.predatorInfluences[gIdx] *= 0.5f;
    buf.predatorThreats[gIdx] = std::max(
        buf.predatorThreats[gIdx] - gSimulationTuning.threatDecay * dt, 0.0f);

    if (buf.stresses[gIdx] > 0.0f) {
      const float decayRate = 1.0f;
      buf.stresses[gIdx] -= easeOut(dt * decayRate);
      if (buf.stresses[gIdx] < 0.0f) {
        buf.stresses[gIdx] = 0.0f;
      }
    }
  }
}
