#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"
#include "boids_buffers.h"
#include "platform_utils.h"
#include "simulation_tuning.h"
#include "species_params.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <numeric>
#include <random>
#include <utility>
#include <vector>
#include <limits>


// グローバル共通
std::vector<SpeciesParams> globalSpeciesParams;
// 静的メンバー変数の初期化
int BoidUnit::nextId = 0;

namespace {
constexpr float kEnvelopeCenterBlend = 0.12f;
constexpr float kEnvelopeRadiusBlend = 0.18f;
constexpr int kMaxClustersPerSpecies = 32;
constexpr float kClusterHistorySeconds = 10.0f;
constexpr int kClusterRetainFrames = 600;
constexpr float kClusterCaptureRadiusScale = 0.4f;
constexpr float kClusterVelocityAlignBias = 0.35f;
// 小クラスター同士を「密集している」と見なすリンク閾値。
// dist <= linkScale * (r_i + r_j) で同一群れ候補とする。
constexpr float kSchoolLinkScale = 1.35f;
constexpr int kMaxSchoolsPerSpecies = 12;
constexpr int kEnvelopeUpdateStride = 4;   // エンベロープ更新（純デバッグ用）は4フレームに1回
constexpr int kTreeRebuildStride = 10;      // 木の再構築間隔を ~0.5秒まで延長

inline float clampf(float value, float minValue, float maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

} // namespace

SpeciesParams BoidTree::getGlobalSpeciesParams(const std::string species) {
  auto it = std::find_if(
      globalSpeciesParams.begin(), globalSpeciesParams.end(),
      [&species](const SpeciesParams &p) { return p.species == species; });
  return (it != globalSpeciesParams.end()) ? *it : SpeciesParams{};
  throw std::invalid_argument("Species not found: " + species);
}

void BoidTree::setGlobalSpeciesParams(const SpeciesParams &params) {
  auto it = std::find_if(globalSpeciesParams.begin(), globalSpeciesParams.end(),
                         [&params](const SpeciesParams &p) {
                           return p.species == params.species;
                         });
  if (it != globalSpeciesParams.end()) {
    *it = params; // 更新
  } else {
    globalSpeciesParams.push_back(params); // 追加
  }
  initializeBoidMemories(globalSpeciesParams);

  // 捕食者の警戒距離（影響範囲）上限を更新。
  // applyPredatorSweep が毎回 globalSpeciesParams を走査すると非常に重いので、
  // 種族パラメータ更新時にここでまとめて再計算してキャッシュする。
  maxPredatorAlertRadius_ = 1.0f;
  for (const auto &p : globalSpeciesParams) {
    if (p.isPredator) {
      continue;
    }
    maxPredatorAlertRadius_ = std::max(maxPredatorAlertRadius_,
                                      std::max(p.predatorAlertRadius, 0.0f));
  }
  speciesClusters.clear();
  speciesClusters.resize(globalSpeciesParams.size());

  speciesSchoolClusters.clear();
  speciesSchoolClusters.resize(globalSpeciesParams.size());

  // 種族構成が変わったのでデバッグ用クラスター表示も作り直し対象
  speciesClusterBufferDirty = true;
}

void BoidTree::updateSpeciesEnvelopes() {
  const std::size_t speciesCount = globalSpeciesParams.size();
  if (speciesCount == 0) {
    speciesEnvelopes.clear();
    speciesEnvelopeBuffer.clear();
    speciesClusters.clear();
    speciesSchoolClusters.clear();
    return;
  }

  speciesEnvelopes.resize(speciesCount);
  if (speciesClusters.size() != speciesCount) {
    speciesClusters.assign(speciesCount, {});
  }
  if (speciesSchoolClusters.size() != speciesCount) {
    speciesSchoolClusters.assign(speciesCount, {});
  }
  std::vector<glm::vec3> accumCenter(speciesCount, glm::vec3(0.0f));
  std::vector<float> accumWeight(speciesCount, 0.0f);

  forEachLeaf([&](const SpatialLeaf &leaf) {
    const BoidUnit *node = leaf.node;
    if (!node) {
      return;
    }
    const int sid = node->speciesId;
    if (sid < 0 || sid >= static_cast<int>(speciesCount)) {
      return;
    }
    const float w = static_cast<float>(leaf.count);
    accumCenter[sid] += node->center * w;
    accumWeight[sid] += w;
  });

  std::vector<glm::vec3> centers(speciesCount, glm::vec3(0.0f));
  for (std::size_t sid = 0; sid < speciesCount; ++sid) {
    if (accumWeight[sid] > 0.0f) {
      centers[sid] = accumCenter[sid] * (1.0f / accumWeight[sid]);
    } else {
      centers[sid] = speciesEnvelopes[sid].center;
    }
  }

  std::vector<float> maxRadius(speciesCount, 0.0f);
  forEachLeaf([&](const SpatialLeaf &leaf) {
    const BoidUnit *node = leaf.node;
    if (!node) {
      return;
    }
    const int sid = node->speciesId;
    if (sid < 0 || sid >= static_cast<int>(speciesCount)) {
      return;
    }
    const glm::vec3 delta = node->center - centers[sid];
    const float dist = glm::length(delta);
    const float r = dist + node->radius;
    if (r > maxRadius[sid]) {
      maxRadius[sid] = r;
    }
  });

  for (std::size_t sid = 0; sid < speciesCount; ++sid) {
    BoidTree::SpeciesEnvelope &env = speciesEnvelopes[sid];
    const glm::vec3 targetCenter = centers[sid];
    const float targetRadius = glm::max(maxRadius[sid], 0.1f);
    const bool hasData = accumWeight[sid] > 0.0f && targetRadius > 0.0f;
    if (hasData) {
      env.center = glm::mix(env.center, targetCenter, kEnvelopeCenterBlend);
      env.radius = glm::mix(env.radius, targetRadius, kEnvelopeRadiusBlend);
      env.count = accumWeight[sid];
    } else {
      env.center = glm::mix(env.center, targetCenter, kEnvelopeCenterBlend);
      env.radius = glm::mix(env.radius, 0.0f, kEnvelopeRadiusBlend * 0.5f);
      env.count = 0.0f;
    }
  }

  const std::size_t packedCount = speciesCount * 5;
  if (speciesEnvelopeBuffer.size() != packedCount) {
    speciesEnvelopeBuffer.resize(packedCount, 0.0f);
  }
  for (std::size_t sid = 0; sid < speciesCount; ++sid) {
    const std::size_t base = sid * 5;
    const BoidTree::SpeciesEnvelope &env = speciesEnvelopes[sid];
    speciesEnvelopeBuffer[base] = env.center.x;
    speciesEnvelopeBuffer[base + 1] = env.center.y;
    speciesEnvelopeBuffer[base + 2] = env.center.z;
    speciesEnvelopeBuffer[base + 3] = env.radius;
    speciesEnvelopeBuffer[base + 4] = env.count;
  }
}

void BoidTree::updateSpeciesClusters(float dt) {
  const std::size_t speciesCount = globalSpeciesParams.size();
  if (speciesCount == 0) {
    speciesClusters.clear();
    speciesClusterBufferDirty = true;
    return;
  }
  if (speciesClusters.size() != speciesCount) {
    speciesClusters.assign(speciesCount, {});
    speciesClusterBufferDirty = true;
  }

  const float safeDt = glm::clamp(dt, 1e-3f, 0.25f);
  // boid 全数ループは重いので、leaf (BoidUnit) 単位で近似集計する。
  // leaf 数はだいたい boidCount/maxBoidsPerUnit 程度なので大幅に軽くなる。
  const int boidCount = getBoidCount();
  if (boidCount <= 0) {
    for (auto &clusters : speciesClusters) {
      for (auto &cluster : clusters) {
        cluster.frameContributionCount = 0;
        cluster.frameSumPosition = glm::vec3(0.0f);
        cluster.frameSumPositionSq = glm::vec3(0.0f);
        cluster.frameSumLeafRadius = 0.0f;
        cluster.frameSumVelocity = glm::vec3(0.0f);
      }
    }
    return;
  }

  for (auto &clusters : speciesClusters) {
    for (auto &cluster : clusters) {
      cluster.frameContributionCount = 0;
      cluster.frameSumPosition = glm::vec3(0.0f);
      cluster.frameSumPositionSq = glm::vec3(0.0f);
      cluster.frameSumLeafRadius = 0.0f;
      cluster.frameSumVelocity = glm::vec3(0.0f);
    }
  }

  forEachLeaf([&](const SpatialLeaf &leaf) {
    const BoidUnit *node = leaf.node;
    if (!node || leaf.count == 0) {
      return;
    }
    const int sid = node->speciesId;
    if (sid < 0 || sid >= static_cast<int>(speciesCount)) {
      return;
    }

    auto &clusters = speciesClusters[sid];
    const SpeciesParams &params = globalSpeciesParams[sid];

    const glm::vec3 pos = node->center;
    const glm::vec3 vel = node->averageVelocity;
    const float leafRadius = glm::max(node->radius, 0.0f);
    const int leafBoidCount = static_cast<int>(glm::min<std::size_t>(leaf.count, 1000000000u));

    float captureRadius = glm::max(
        params.cohesionRange * kClusterCaptureRadiusScale,
        params.separationRange * 1.35f);
    // leaf 集計では中心が粗くなるため少し小さめにして誤吸い込みを抑える
    captureRadius = glm::clamp(captureRadius, 2.0f, 240.0f);
    const float captureRadiusSq = captureRadius * captureRadius;

    int bestIndex = -1;
    float bestScore = captureRadiusSq;
    const float velLen2 = glm::length2(vel);
    const glm::vec3 velDir = velLen2 > 1e-6f
                                 ? vel * (1.0f / glm::sqrt(velLen2))
                                 : glm::vec3(0.0f);

    for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
      const auto &cluster = clusters[c];
      if (!cluster.active) {
        continue;
      }
      const glm::vec3 diff = pos - cluster.center;
      const float distSq = glm::dot(diff, diff);
      if (distSq > captureRadiusSq) {
        continue;
      }
      float score = distSq;
      const float clusterVelLen2 = glm::length2(cluster.avgVelocity);
      if (clusterVelLen2 > 1e-6f && velLen2 > 1e-6f) {
        const glm::vec3 clusterVelDir =
            cluster.avgVelocity * (1.0f / glm::sqrt(clusterVelLen2));
        const float align = glm::clamp(glm::dot(clusterVelDir, velDir), -1.0f, 1.0f);
        const float alignBias = glm::mix(1.0f + kClusterVelocityAlignBias,
                                         1.0f - kClusterVelocityAlignBias,
                                         (align + 1.0f) * 0.5f);
        score *= glm::clamp(alignBias, 0.4f, 1.6f);
      }
      if (score < bestScore) {
        bestScore = score;
        bestIndex = c;
      }
    }

    if (bestIndex < 0) {
      if (clusters.size() < kMaxClustersPerSpecies) {
        SpeciesCluster cluster;
        cluster.center = pos;
        cluster.avgVelocity = vel;
        cluster.radius = glm::clamp(glm::max(params.separationRange * 2.0f, leafRadius * 1.25f),
                                    1.0f, 120.0f);
        cluster.weight = static_cast<float>(leafBoidCount);
        cluster.lastUpdateFrame = frameCount;
        cluster.active = true;
        clusters.push_back(cluster);
        bestIndex = static_cast<int>(clusters.size()) - 1;
      } else if (!clusters.empty()) {
        int weakest = 0;
        float weakestWeight = clusters[0].weight;
        for (int c = 1; c < static_cast<int>(clusters.size()); ++c) {
          if (clusters[c].weight < weakestWeight) {
            weakestWeight = clusters[c].weight;
            weakest = c;
          }
        }
        SpeciesCluster &cluster = clusters[weakest];
        cluster.center = pos;
        cluster.avgVelocity = vel;
        cluster.radius = glm::clamp(glm::max(params.separationRange * 2.0f, leafRadius * 1.25f),
                                    1.0f, 120.0f);
        cluster.weight = static_cast<float>(leafBoidCount);
        cluster.lastUpdateFrame = frameCount;
        cluster.active = true;
        cluster.frameContributionCount = 0;
        cluster.frameSumPosition = glm::vec3(0.0f);
        cluster.frameSumPositionSq = glm::vec3(0.0f);
        cluster.frameSumLeafRadius = 0.0f;
        cluster.frameSumVelocity = glm::vec3(0.0f);
        bestIndex = weakest;
      } else {
        return;
      }
    }

    SpeciesCluster &assigned = speciesClusters[sid][bestIndex];
    const float w = static_cast<float>(leafBoidCount);
    assigned.frameSumPosition += pos * w;
    assigned.frameSumPositionSq += (pos * pos) * w;
    assigned.frameSumLeafRadius += leafRadius * w;
    assigned.frameSumVelocity += vel * w;
    assigned.frameContributionCount += leafBoidCount;
  });

  const float decayPerFrame = glm::clamp(safeDt / kClusterHistorySeconds, 0.002f,
                                         0.25f);
  for (std::size_t sid = 0; sid < speciesCount; ++sid) {
    const SpeciesParams &params = globalSpeciesParams[sid];
    auto &clusters = speciesClusters[sid];
    for (auto &cluster : clusters) {
      if (cluster.frameContributionCount > 0) {
        const float invCount =
            1.0f / static_cast<float>(cluster.frameContributionCount);
        const glm::vec3 samplePos = cluster.frameSumPosition * invCount;
        const glm::vec3 samplePosSq = cluster.frameSumPositionSq * invCount;
        const glm::vec3 sampleVel = cluster.frameSumVelocity * invCount;
        const float meanLeafRadius =
            cluster.frameSumLeafRadius > 0.0f ? (cluster.frameSumLeafRadius * invCount) : 0.0f;

        // 分散から RMS 半径を推定して「実際の塊の大きさ」を追従させる。
        // var = E[x^2] - (E[x])^2（成分ごと） → std = sqrt(var.x+var.y+var.z)
        glm::vec3 var = samplePosSq - (samplePos * samplePos);
        var = glm::max(var, glm::vec3(0.0f));
        const float stdRadius = glm::sqrt(var.x + var.y + var.z) + meanLeafRadius;
        // だいたい 2.5σ + 近接レンジ分を、群れの見た目半径として使う
        const float radiusTarget = glm::clamp(
            stdRadius * 2.5f + glm::max(params.separationRange * 0.5f, 0.25f),
            0.75f, 240.0f);
        const float hitBlend = glm::clamp(
            decayPerFrame * static_cast<float>(cluster.frameContributionCount),
            0.05f, 0.85f);
        cluster.center = glm::mix(cluster.center, samplePos, hitBlend);
        cluster.avgVelocity = glm::mix(cluster.avgVelocity, sampleVel, hitBlend);
        cluster.radius = glm::mix(cluster.radius, radiusTarget, 0.25f);
        cluster.weight =
            glm::clamp(cluster.weight + static_cast<float>(cluster.frameContributionCount),
                       0.0f, 50000.0f);
        cluster.lastUpdateFrame = frameCount;
        cluster.active = true;
      } else {
        cluster.weight *= glm::clamp(1.0f - decayPerFrame * 1.3f, 0.0f, 1.0f);
        if ((frameCount - cluster.lastUpdateFrame) > kClusterRetainFrames ||
            cluster.weight < 0.25f) {
          cluster.active = false;
        }
      }
      cluster.frameContributionCount = 0;
      cluster.frameSumPosition = glm::vec3(0.0f);
      cluster.frameSumPositionSq = glm::vec3(0.0f);
      cluster.frameSumLeafRadius = 0.0f;
      cluster.frameSumVelocity = glm::vec3(0.0f);
    }
  }

  // 中身が更新されたので、次に JS が読むタイミングで再パックする
  speciesClusterBufferDirty = true;
}

const BoidTree::SpeciesEnvelope *BoidTree::getSpeciesEnvelope(
    int speciesId) const {
  if (speciesId < 0 || speciesId >= static_cast<int>(speciesEnvelopes.size())) {
    return nullptr;
  }
  return &speciesEnvelopes[speciesId];
}

const std::vector<BoidTree::SpeciesCluster> *
BoidTree::getSpeciesClusters(int speciesId) const {
  if (speciesId < 0 || speciesId >= static_cast<int>(speciesClusters.size())) {
    return nullptr;
  }
  return &speciesClusters[speciesId];
}

uintptr_t BoidTree::getSpeciesEnvelopePtr() {
  if (speciesEnvelopeBuffer.empty()) {
    return 0;
  }
  return reinterpret_cast<uintptr_t>(speciesEnvelopeBuffer.data());
}

int BoidTree::getSpeciesEnvelopeCount() const {
  return static_cast<int>(speciesEnvelopeBuffer.size());
}

void BoidTree::rebuildSpeciesClusterDebugBuffer() {
  speciesClusterBuffer.clear();

  // デバッグ用途なので、active クラスターだけをパックして送る。
  // ここはクラスター数が最大でも species*32 程度なので、毎フレームでも十分軽いが、
  // 表示OFF時の無駄を避けるため getSpeciesClustersPtr() から遅延構築する。
  if (speciesClusters.empty()) {
    return;
  }

  // ざっくりした上限で reserve（不要な再確保を減らす）
  std::size_t reserveFloats = 0;
  for (const auto &clusters : speciesClusters) {
    reserveFloats += clusters.size() * 6;
  }
  speciesClusterBuffer.reserve(reserveFloats);

  for (std::size_t sid = 0; sid < speciesClusters.size(); ++sid) {
    const auto &clusters = speciesClusters[sid];
    for (const auto &cluster : clusters) {
      if (!cluster.active) {
        continue;
      }

      speciesClusterBuffer.push_back(static_cast<float>(sid));
      speciesClusterBuffer.push_back(cluster.center.x);
      speciesClusterBuffer.push_back(cluster.center.y);
      speciesClusterBuffer.push_back(cluster.center.z);
      speciesClusterBuffer.push_back(cluster.radius);
      speciesClusterBuffer.push_back(cluster.weight);
    }
  }
}

void BoidTree::rebuildSpeciesSchoolClusterDebugBuffer() {
  speciesSchoolClusterBuffer.clear();

  // デバッグ用途なので active の群れ（大クラスター）だけをパックして送る。
  // 群れ数は小さい（species*12 程度）ため、構築コストは軽いが、
  // 表示OFF時の無駄を避けるため getSpeciesSchoolClustersPtr() から遅延構築する。
  if (speciesSchoolClusters.empty()) {
    return;
  }

  // ざっくりした上限で reserve（不要な再確保を減らす）
  std::size_t reserveFloats = 0;
  for (const auto &schools : speciesSchoolClusters) {
    reserveFloats += schools.size() * 6;
  }
  speciesSchoolClusterBuffer.reserve(reserveFloats);

  for (std::size_t sid = 0; sid < speciesSchoolClusters.size(); ++sid) {
    const auto &schools = speciesSchoolClusters[sid];
    for (const auto &school : schools) {
      if (!school.active) {
        continue;
      }

      speciesSchoolClusterBuffer.push_back(static_cast<float>(sid));
      speciesSchoolClusterBuffer.push_back(school.center.x);
      speciesSchoolClusterBuffer.push_back(school.center.y);
      speciesSchoolClusterBuffer.push_back(school.center.z);
      speciesSchoolClusterBuffer.push_back(school.radius);
      speciesSchoolClusterBuffer.push_back(school.weight);
    }
  }
}

uintptr_t BoidTree::getSpeciesClustersPtr() {
  if (speciesClusterBufferDirty) {
    rebuildSpeciesClusterDebugBuffer();
    speciesClusterBufferDirty = false;
  }
  if (speciesClusterBuffer.empty()) {
    return 0;
  }
  return reinterpret_cast<uintptr_t>(speciesClusterBuffer.data());
}

int BoidTree::getSpeciesClustersCount() const {
  return static_cast<int>(speciesClusterBuffer.size());
}

uintptr_t BoidTree::getSpeciesSchoolClustersPtr() {
  if (speciesSchoolClusterBufferDirty) {
    rebuildSpeciesSchoolClusterDebugBuffer();
    speciesSchoolClusterBufferDirty = false;
  }
  if (speciesSchoolClusterBuffer.empty()) {
    return 0;
  }
  return reinterpret_cast<uintptr_t>(speciesSchoolClusterBuffer.data());
}

int BoidTree::getSpeciesSchoolClustersCount() const {
  return static_cast<int>(speciesSchoolClusterBuffer.size());
}

BoidTree::BoidTree()
    : root(nullptr), frameCount(0), splitIndex(0), mergeIndex(0),
      maxBoidsPerUnit(10) {}

BoidTree::~BoidTree() {
  if (root) {
    returnNodeToPool(root);
    root = nullptr;
  }
  clearPool();
}

// プール管理
BoidUnit *BoidTree::getUnitFromPool() {
  if (!unitPool.empty()) {
    BoidUnit *unit = unitPool.top();
    unitPool.pop();

  // リセット
    unit->children.clear();
    unit->indices.clear();
    unit->center = glm::vec3(0.0f);
    unit->averageVelocity = glm::vec3(0.0f);
    unit->radius = 0.0f;
    unit->simpleDensity = 0.0f;
    unit->frameCount = 0;
    unit->speciesId = -1;
    unit->buf = &buf;

    return unit;
  }

  // プールが空の場合は新規作成
  BoidUnit *unit = new BoidUnit();
  unit->buf = &buf;
  unit->simpleDensity = 0.0f;
  return unit;
}

void BoidTree::returnUnitToPool(BoidUnit *unit) {
  if (!unit)
    return;

  // NOTE:
  // ここが再帰のままだと、ツリー破損（循環参照）や想定外の深さで
  // ブラウザ側で RangeError("Maximum call stack size exceeded") が発生し得る。
  // 反復DFS＋安全弁で、クラッシュを避ける。
  // NOTE: ここは頻繁に呼ばれる可能性があるので、thread_local でスタックを再利用し、
  // allocator負荷（vectorの確保/破棄）を抑える。
  static thread_local std::vector<std::pair<BoidUnit *, BoidUnit *>> stack;
  stack.clear();
  if (stack.capacity() < 256) {
    stack.reserve(256);
  }
  stack.emplace_back(unit, nullptr);

  constexpr std::size_t kMaxTraversalSteps = 5'000'000;
  std::size_t steps = 0;

  while (!stack.empty()) {
    const auto [current, parent] = stack.back();
    stack.pop_back();
    if (!current) {
      continue;
    }
    if (++steps > kMaxTraversalSteps) {
      return;
    }

    for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
      BoidUnit *child = *it;
      if (!child) {
        continue;
      }
      if (child == current || child == parent) {
        continue;
      }
      stack.emplace_back(child, current);
    }
    current->children.clear();
    unitPool.push(current);
  }
}

void BoidTree::returnNodeToPool(BoidUnit *node) {
  // returnUnitToPool と同等だが、過去互換のため残している。
  returnUnitToPool(node);
}

void BoidTree::forEachLeafRecursive(const BoidUnit *node,
                                    const LeafVisitor &visitor) const {
  // NOTE:
  // WASM で深い再帰を行うと、ブラウザ側で RangeError("Maximum call stack size exceeded")
  // が発生し得る（JSスタック/ランタイム実装の都合で例外として表面化する）。
  // ここは反復DFSにして、ツリーの深さに依存しない走査にする。
  if (!node) {
    return;
  }

  // parent を保持して簡易的に循環参照（親へ戻る/自己参照）を避ける。
  // NOTE: フレーム内で多回呼ばれるため、thread_local でスタックを再利用して
  // allocator負荷（vectorの確保/破棄）を抑える。
  static thread_local std::vector<std::pair<const BoidUnit *, const BoidUnit *>> stack;
  stack.clear();
  if (stack.capacity() < 256) {
    stack.reserve(256);
  }
  stack.emplace_back(node, nullptr);

  // 異常系（循環や破損）で無限ループにならないための安全弁。
  constexpr std::size_t kMaxTraversalSteps = 5'000'000;
  std::size_t steps = 0;

  while (!stack.empty()) {
    const auto [current, parent] = stack.back();
    stack.pop_back();
    if (!current) {
      continue;
    }
    if (++steps > kMaxTraversalSteps) {
      // ここに到達するのはツリー破損などの異常ケース。
      // 走査を打ち切ってクラッシュを回避する。
      return;
    }

    if (current->children.empty()) {
      SpatialLeaf leaf{current->indices.data(), current->indices.size(), current};
      visitor(leaf);
      continue;
    }

    // push はLIFOなので、順序を保ちたい場合は逆順push。
    for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
      const BoidUnit *child = *it;
      if (!child) {
        continue;
      }
      if (child == current || child == parent) {
        continue;
      }
      stack.emplace_back(child, current);
    }
  }
}

void BoidTree::forEachLeafIntersectingSphereRecursive(
    const BoidUnit *node, const glm::vec3 &center, float radius,
    const LeafVisitor &visitor) const {
  // こちらも forEachLeafRecursive と同様に反復DFS化。
  if (!node) {
    return;
  }

  static thread_local std::vector<std::pair<const BoidUnit *, const BoidUnit *>> stack;
  stack.clear();
  if (stack.capacity() < 256) {
    stack.reserve(256);
  }
  stack.emplace_back(node, nullptr);

  constexpr std::size_t kMaxTraversalSteps = 5'000'000;
  std::size_t steps = 0;

  while (!stack.empty()) {
    const auto [current, parent] = stack.back();
    stack.pop_back();
    if (!current) {
      continue;
    }
    if (++steps > kMaxTraversalSteps) {
      return;
    }

    const glm::vec3 delta = current->center - center;
    const float maxDist = current->radius + radius;
    if (glm::dot(delta, delta) > maxDist * maxDist) {
      continue;
    }

    if (current->children.empty()) {
      SpatialLeaf leaf{current->indices.data(), current->indices.size(), current};
      visitor(leaf);
      continue;
    }

    for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
      const BoidUnit *child = *it;
      if (!child) {
        continue;
      }
      if (child == current || child == parent) {
        continue;
      }
      stack.emplace_back(child, current);
    }
  }
}

void BoidTree::clearPool() {
  while (!unitPool.empty()) {
    delete unitPool.top();
    unitPool.pop();
  }
}

void BoidTree::forEachLeaf(const LeafVisitor &visitor) const {
  if (!root || !visitor) {
    return;
  }
  forEachLeafRecursive(root, visitor);
}

void BoidTree::forEachLeafIntersectingSphere(const glm::vec3 &center,
                                             float radius,
                                             const LeafVisitor &visitor) const {
  if (!root || !visitor) {
    return;
  }
  forEachLeafIntersectingSphereRecursive(root, center, radius, visitor);
}

// ダブルバッファのRead側をレンダリング用ポインタに設定
void BoidTree::setRenderPointersToReadBuffers() {
  renderPositionsPtr_ = buf.positions.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(
                                  buf.positions.data());
  renderVelocitiesPtr_ = buf.velocities.empty()
                             ? 0
                             : reinterpret_cast<uintptr_t>(
                                   buf.velocities.data());
  renderOrientationsPtr_ = buf.orientations.empty()
                               ? 0
                               : reinterpret_cast<uintptr_t>(
                                     buf.orientations.data());
}

// ダブルバッファのWrite側をレンダリング用ポインタに設定
void BoidTree::setRenderPointersToWriteBuffers() {
  renderPositionsPtr_ = buf.positionsWrite.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(
                                  buf.positionsWrite.data());
  renderVelocitiesPtr_ = buf.velocitiesWrite.empty()
                             ? 0
                             : reinterpret_cast<uintptr_t>(
                                   buf.velocitiesWrite.data());
  renderOrientationsPtr_ = buf.orientationsWrite.empty()
                               ? 0
                               : reinterpret_cast<uintptr_t>(
                                     buf.orientationsWrite.data());
}

void BoidTree::build(int maxPerUnit) {
  // 既存の root を削除して再生成
  if (root) {
    returnNodeToPool(root);
  }
  root = getUnitFromPool();
  root->buf = &buf; // 中央バッファを共有

  maxBoidsPerUnit = maxPerUnit;

  // すべての Boid インデックスを作成
  std::vector<int> indices(buf.positions.size());
  std::iota(indices.begin(), indices.end(), 0);

  buildRecursive(root, indices, maxPerUnit);
  // printTree(root, 0);
  setRenderPointersToReadBuffers();
}

// NOTE:
// 過去に build(maxPerUnit, level) も存在しましたが、現在は build(maxPerUnit) に統一しています。
// インデックス構築の呼び出し側(API)を単純化し、誤解を減らすのが目的です。

// 既存のユニットを再利用して木構造を再構築する関数
void BoidTree::rebuildTreeWithUnits(BoidUnit *node,
                                    const std::vector<BoidUnit *> &units,
                                    int maxPerUnit) {

  // 既存の children をクリア
  node->children.clear();

  // 空間的に分割
  if ((int)units.size() <= maxPerUnit) {
    // 葉ノードとしてユニットを直接保持
    for (auto *unit : units) {
      // 必要な変数をリセットしつつ indices を保持
      unit->frameCount = 0;

      node->children.push_back(unit);
    }
    node->computeBoundingSphere(); // バウンディングスフィアを計算
    return;
  }

  // 分割軸を決定
  float mean[3] = {0}, var[3] = {0};
  for (const auto *unit : units) {
    const glm::vec3 &p = unit->center;
    mean[0] += p.x;
    mean[1] += p.y;
    mean[2] += p.z;
  }
  for (int k = 0; k < 3; ++k)
    mean[k] /= units.size();

  for (const auto *unit : units) {
    const glm::vec3 &p = unit->center;
    var[0] += (p.x - mean[0]) * (p.x - mean[0]);
    var[1] += (p.y - mean[1]) * (p.y - mean[1]);
    var[2] += (p.z - mean[2]) * (p.z - mean[2]);
  }
  int axis = (var[1] > var[0]) ? 1 : 0;
  if (var[2] > var[axis])
    axis = 2;

  // ユニットを分割
  std::vector<BoidUnit *> sortedUnits = units;
  std::sort(sortedUnits.begin(), sortedUnits.end(),
            [axis](BoidUnit *a, BoidUnit *b) {
              const glm::vec3 &pa = a->center;
              const glm::vec3 &pb = b->center;
              return (axis == 0)   ? pa.x < pb.x
                     : (axis == 1) ? pa.y < pb.y
                                   : pa.z < pb.z;
            });
  std::size_t mid = sortedUnits.size() / 2;
  std::vector<BoidUnit *> left(sortedUnits.begin(), sortedUnits.begin() + mid);
  std::vector<BoidUnit *> right(sortedUnits.begin() + mid, sortedUnits.end());

  auto *leftChild = getUnitFromPool();
  auto *rightChild = getUnitFromPool();
  leftChild->buf = node->buf;
  rightChild->buf = node->buf;

  node->children.push_back(leftChild);
  node->children.push_back(rightChild);

  rebuildTreeWithUnits(leftChild, left, maxPerUnit);
  rebuildTreeWithUnits(rightChild, right, maxPerUnit);

  node->computeBoundingSphere(); // バウンディングスフィアを計算
}

void BoidTree::buildRecursive(BoidUnit *node, const std::vector<int> &indices,
                              int maxPerUnit) {

  // 既存の children をクリア
  node->children.clear();

  // 個体が存在しない場合は安全に早期リターン（メモリアクセス違反を防止）
  if (indices.empty()) {
    node->indices.clear();
    node->speciesId = -1;
    node->center = glm::vec3(0.0f);
    node->averageVelocity = glm::vec3(0.0f);
    node->radius = 0.0f;
    return;
  }

  // 末端ノード（葉）の処理
  if ((int)indices.size() <= maxPerUnit) {
    node->indices = indices;

    // 種が 1 種類かどうかを判定
    int firstSpeciesId = buf.speciesIds[indices[0]];
    bool mixedSpecies = false;
    for (int i : indices) {
      if (buf.speciesIds[i] != firstSpeciesId) {
        mixedSpecies = true;
        break;
      }
    }

    if (!mixedSpecies) {
      // 種が 1 種類の場合
      node->speciesId = firstSpeciesId;
    } else {
      // 種が混在している場合、種ごとに再分割（線形バケット化）
      // 最大種数を想定して固定サイズ配列を使用（O(N)処理）
      const int MAX_SPECIES = 16;
      std::vector<int> speciesBuckets[MAX_SPECIES];
      std::vector<int> usedSpecies;

      // 線形スキャンで種別バケット化
      for (int i : indices) {
        int speciesId = buf.speciesIds[i];
        if (speciesId >= 0 && speciesId < MAX_SPECIES) {
          if (speciesBuckets[speciesId].empty()) {
            usedSpecies.push_back(speciesId);
          }
          speciesBuckets[speciesId].push_back(i);
        }
      }

      // 使用された種別のみ処理
      for (int speciesId : usedSpecies) {
        const std::vector<int> &groupIndices = speciesBuckets[speciesId];

        auto *child = getUnitFromPool();
        child->buf = node->buf;
        child->indices = groupIndices;
        child->speciesId = speciesId;

        for (int gIdx : groupIndices) {
          buf.speciesIds[gIdx] = speciesId;
        }

        child->computeBoundingSphere();
        node->children.push_back(child);
      }
    }

    node->computeBoundingSphere();
    return;
  }

  // 空間的に分割
  float mean[3] = {0}, var[3] = {0};
  for (int i : indices) {
    const glm::vec3 &p = buf.positions[i];
    mean[0] += p.x;
    mean[1] += p.y;
    mean[2] += p.z;
  }
  for (int k = 0; k < 3; ++k)
    mean[k] /= indices.size();

  for (int i : indices) {
    const glm::vec3 &p = buf.positions[i];
    var[0] += (p.x - mean[0]) * (p.x - mean[0]);
    var[1] += (p.y - mean[1]) * (p.y - mean[1]);
    var[2] += (p.z - mean[2]) * (p.z - mean[2]);
  }
  int axis = (var[1] > var[0]) ? 1 : 0;
  if (var[2] > var[axis])
    axis = 2;

  // 線形パーティション処理（ソート回避）
  std::vector<int> left, right;

  // 軸に沿った最小値と最大値を求める
  float minVal = FLT_MAX, maxVal = -FLT_MAX;
  for (int i : indices) {
    const glm::vec3 &p = buf.positions[i];
    float val = (axis == 0) ? p.x : (axis == 1) ? p.y : p.z;
    minVal = std::min(minVal, val);
    maxVal = std::max(maxVal, val);
  }

  // 中点で分割
  float midVal = (minVal + maxVal) * 0.5f;

  // 線形パーティション
  for (int i : indices) {
    const glm::vec3 &p = buf.positions[i];
    float val = (axis == 0) ? p.x : (axis == 1) ? p.y : p.z;
    if (val < midVal) {
      left.push_back(i);
    } else {
      right.push_back(i);
    }
  }

  // 極端な偏りの場合のフォールバック（50/50に近い分割を強制）
  if (left.empty() || right.empty() || (left.size() > indices.size() * 0.8f) ||
      (right.size() > indices.size() * 0.8f)) {
    // nth_elementで中央値分割
    std::vector<int> sorted = indices;
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2,
                     sorted.end(), [this, axis](int a, int b) {
                       const glm::vec3 &pa = buf.positions[a];
                       const glm::vec3 &pb = buf.positions[b];
                       return (axis == 0)   ? pa.x < pb.x
                              : (axis == 1) ? pa.y < pb.y
                                            : pa.z < pb.z;
                     });
    std::size_t mid = sorted.size() / 2;
    left.assign(sorted.begin(), sorted.begin() + mid);
    right.assign(sorted.begin() + mid, sorted.end());
  }

  auto *leftChild = getUnitFromPool();
  auto *rightChild = getUnitFromPool();
  leftChild->buf = node->buf;
  rightChild->buf = node->buf;

  node->children.push_back(leftChild);
  node->children.push_back(rightChild);

  buildRecursive(leftChild, left, maxPerUnit);
  buildRecursive(rightChild, right, maxPerUnit);

  node->speciesId = -1; // 内部ノードは種を持たない
  node->computeBoundingSphere();
}

void BoidTree::update(float dt) {
  // NOTE:
  // dt が NaN/Inf/負だと、全個体が同じ汚染を共有して「全体ガガガ/全体停止」になりやすい。
  // ここは1フレーム1回の防波堤としてコスト無視できる。
  if (!std::isfinite(dt) || dt < 0.0f) {
    dt = 0.0f;
  }

  if ((frameCount % kEnvelopeUpdateStride) == 0) {
    updateSpeciesEnvelopes();
  }

  // 木構造全体を再帰的に更新
  if (root) {
    try {
      setRenderPointersToReadBuffers();
      root->updateRecursive(glm::clamp(dt, 0.0f, 0.1f) * 5);
      if (dt > 0.0f) {
        setRenderPointersToWriteBuffers();
        buf.swapReadWrite();
        setRenderPointersToReadBuffers();
      }
    } catch (const std::exception &e) {
      std::cerr << "Exception caught: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Unknown exception caught" << std::endl;
    }
  } else {
    setRenderPointersToReadBuffers();
  }

  // クラスター更新は重いので、数フレームに1回だけ行う。
  // ただし EMA の時間スケールは保ちたいので dt は蓄積してからまとめて渡す。
  constexpr int kClusterUpdateStride = 3;
  clusterUpdateDtAccumulator_ += dt;
  if ((frameCount % kClusterUpdateStride) == 0) {
    const float clusteredDt = clusterUpdateDtAccumulator_;
    clusterUpdateDtAccumulator_ = 0.0f;

    updateSpeciesClusters(clusteredDt);
    // 小クラスターを素材に、より大きい「群れ」中心を推定（10秒EMAで安定化）。
    updateSpeciesSchoolClusters(clusteredDt);
  }
  frameCount++;

  // 一定フレームごとに葉ノードを再収集
  if (frameCount % 15 == 0) { // 15フレーム（約0.25秒）ごとに収集
    leafCache.clear();
    if (root) {
      collectLeavesForCache(root, nullptr);
    }
    splitIndex = 0;
    mergeIndex = 0;
  }

  // 一定フレームごとに木構造を再構築（大幅に頻度を減らす）
  if ((frameCount % kTreeRebuildStride) == 0) {
    build(maxBoidsPerUnit);
    // printTree(root, 0); // ツリー構造をログに出力

    // NOTE:
    // build() は root 以下のノードをプールに返却して再利用する。
    // そのため、再構築前に集めた leafCache の node/parent ポインタは全て無効になる。
    // これをクリアしないと、後段の split/merge が「返却済みノード」を触って
    // メモリ破壊 → 全体ガガガ/停止 になり得る。
    leafCache.clear();
    splitIndex = 0;
    mergeIndex = 0;
  }

  // 分割と結合の処理
  if (!leafCache.empty()) {
    // 分割
    for (int i = 0; i < 12 && splitIndex < (int)leafCache.size();
         ++i, ++splitIndex) {
      BoidUnit *u = leafCache[splitIndex].node;
      if (u && u->needsSplit(40.0f, 0.5f, maxBoidsPerUnit)) {
        u->splitInPlace(maxBoidsPerUnit);
        leafCache.clear();
        break;
      }
    }

    // 結合
    for (int i = 0; i < 12 && mergeIndex < (int)leafCache.size();
         ++i, ++mergeIndex) {
      for (int j = mergeIndex + 1; j < (int)leafCache.size(); ++j) {
        BoidUnit *a = leafCache[mergeIndex].node;
        BoidUnit *b = leafCache[j].node;
        BoidUnit *parent = leafCache[j].parent;
        if (a && b && parent && a->canMergeWith(*b)) {
          leafCache.clear();
          a->mergeWith(b);
          // 親子リンクを遡らずに葉ノードを除去し、プールに戻す
          auto it =
              std::find(parent->children.begin(), parent->children.end(), b);
            if (it != parent->children.end()) {
              parent->children.erase(it);
            }
            returnNodeToPool(b);
          break;
        }
      }
    }
  }

}

// 分割判定を局所的に適用
void BoidTree::trySplitRecursive(BoidUnit *node) {
  if (!node)
    return;
  if (node->isBoidUnit() && node->needsSplit(40.0f, 0.5f, maxBoidsPerUnit)) {
    node->splitInPlace(maxBoidsPerUnit);
  }
  for (auto *c : node->children)
    trySplitRecursive(c);
}
void BoidTree::initializeBoids(
    const std::vector<SpeciesParams> &speciesParamsList, float posRange,
    float velRange) {
  // globalSpeciesParams を更新
  try {
    globalSpeciesParams = speciesParamsList; // コピー操作
  } catch (const std::length_error &e) {
    std::cerr << "Error updating globalSpeciesParams: " << e.what()
              << std::endl;
    return;
  }

  // 全体の個体数を計算
  int totalCount = 0;
  for (const auto &species : globalSpeciesParams) {
    totalCount += species.count;
  }

  // バッファを確保（読み／書きの両方をゼロ初期化）
  buf.reserveAll(totalCount);
  buf.resizeAll(totalCount);
  std::fill(buf.accelerations.begin(), buf.accelerations.end(), glm::vec3(0.0f));
  std::fill(buf.predatorInfluences.begin(), buf.predatorInfluences.end(), glm::vec3(0.0f));
  std::fill(buf.stresses.begin(), buf.stresses.end(), 0.0f);
  std::fill(buf.predatorTargetIndices.begin(), buf.predatorTargetIndices.end(), -1);
  std::fill(buf.predatorTargetTimers.begin(), buf.predatorTargetTimers.end(), 0.0f);
  std::fill(buf.predatorRestTimers.begin(), buf.predatorRestTimers.end(), 0.0f);
  std::fill(buf.predatorChaseTimers.begin(), buf.predatorChaseTimers.end(), 0.0f);
  std::fill(buf.predatorApproachDirs.begin(), buf.predatorApproachDirs.end(), glm::vec3(0.0f));
  std::fill(buf.predatorDisengageDirs.begin(), buf.predatorDisengageDirs.end(), glm::vec3(0.0f));
  std::fill(buf.velocitiesWrite.begin(), buf.velocitiesWrite.end(), glm::vec3(0.0f));
  std::fill(buf.positionsWrite.begin(), buf.positionsWrite.end(), glm::vec3(0.0f));
  std::fill(buf.orientationsWrite.begin(), buf.orientationsWrite.end(), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

  // 各種族の個体を生成
  int offset = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> posDist(-posRange, posRange);
  std::uniform_real_distribution<float> velDist(-velRange, velRange);

  for (size_t speciesId = 0; speciesId < globalSpeciesParams.size();
       ++speciesId) {
    const auto &species = globalSpeciesParams[speciesId];

    // 初期化時に乱数で速度を与えると、有限個サンプルの偏りで
    // 種ごとの平均速度（重心ドリフト）が残ることがある。
    // alignment と minSpeed の組み合わせでその偏りが増幅されると、
    // 「全員が同じ方向に向かって遠くへ行く」挙動になりやすい。
    // ここでは種ごとに平均速度を引いて、並進だけを打ち消しておく。
    glm::vec3 speciesVelocitySum(0.0f);
    const int speciesBegin = offset;

    for (int i = 0; i < species.count; ++i) {
      buf.positions[offset] =
          glm::vec3(posDist(gen), posDist(gen), posDist(gen));
      const glm::vec3 v = glm::vec3(velDist(gen), velDist(gen), velDist(gen));
      buf.velocities[offset] = v;
      speciesVelocitySum += v;
      buf.ids[offset] = offset;
      buf.speciesIds[offset] = speciesId;
      ++offset;
    }

    const int speciesCount = species.count;
    if (speciesCount > 0) {
      const glm::vec3 meanVelocity = speciesVelocitySum / float(speciesCount);
      for (int i = 0; i < speciesCount; ++i) {
        buf.velocities[speciesBegin + i] -= meanVelocity;
      }
    }
  }

  buf.syncWriteFromRead();
  setRenderPointersToReadBuffers();

  // 木構造を再構築
  if (root)
    returnNodeToPool(root);
  root = getUnitFromPool();
  root->buf = &buf;

  std::vector<int> indices(totalCount);
  std::iota(indices.begin(), indices.end(), 0);
  buildRecursive(root, indices, maxBoidsPerUnit);

  // BoidメモリーとActiveNeighborsを初期化
  initializeBoidMemories(globalSpeciesParams);
}

// BoidTree::setFlockSize
void BoidTree::setFlockSize(int newSize, float posRange, float velRange) {
  int current = static_cast<int>(buf.positions.size());

  // 個体を減らす
  if (newSize < current) {
    buf.positions.resize(newSize);
    buf.positionsWrite.resize(newSize);
    buf.velocities.resize(newSize);
    buf.velocitiesWrite.resize(newSize);
    buf.accelerations.resize(newSize);
    buf.ids.resize(newSize);
    buf.stresses.resize(newSize);
    buf.speciesIds.resize(newSize);
    buf.orientations.resize(newSize);
    buf.orientationsWrite.resize(newSize);
    buf.predatorTargetIndices.resize(newSize, -1);
    buf.predatorTargetTimers.resize(newSize, 0.0f);
    buf.predatorRestTimers.resize(newSize, 0.0f);
    buf.predatorChaseTimers.resize(newSize, 0.0f);
    buf.boidCohesionMemories.resize(newSize);
    buf.boidActiveNeighbors.resize(newSize);
    buf.boidNeighborIndices.resize(newSize);

  }
  // 個体を増やす
  else if (newSize > current) {
    int addN = newSize - current;
    buf.reserveAll(newSize);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int k = 0; k < addN; ++k) {
      int i = current + k;
      const glm::vec3 pos =
          glm::vec3(posDist(gen), posDist(gen), posDist(gen));
      const glm::vec3 vel =
          glm::vec3(velDist(gen), velDist(gen), velDist(gen));
      buf.positions.push_back(pos);
      buf.positionsWrite.push_back(pos);
      buf.velocities.push_back(vel);
      buf.velocitiesWrite.push_back(vel);
      buf.accelerations.push_back(glm::vec3(0.0f));
      buf.ids.push_back(i);
      buf.stresses.push_back(0.0f);
      buf.speciesIds.push_back(0);
      buf.orientations.push_back(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      buf.orientationsWrite.push_back(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      buf.predatorTargetIndices.push_back(-1);
      buf.predatorTargetTimers.push_back(0.0f);
      buf.predatorRestTimers.push_back(0.0f);
      buf.predatorChaseTimers.push_back(0.0f);
    }

    // 追加生成時も平均速度の偏りが出ると、群れ全体が一方向へ流れやすい。
    // 既存個体も含めた全体平均を引いて、並進ドリフトを抑える。
    if (!buf.velocities.empty()) {
      glm::vec3 velocitySum(0.0f);
      for (const auto &v : buf.velocities) {
        velocitySum += v;
      }
      const glm::vec3 meanVelocity = velocitySum / float(buf.velocities.size());
      for (auto &v : buf.velocities) {
        v -= meanVelocity;
      }
      // velocitiesWrite は後段の syncWriteFromRead() で同期される。
    }

    // 新しいBoidのメモリを初期化
    buf.boidCohesionMemories.resize(newSize);
    buf.boidActiveNeighbors.resize(newSize);
    buf.boidNeighborIndices.resize(newSize);

    // 新しく追加されたBoidのメモリを初期化（デフォルトのmaxNeighbors=4を使用）
    for (int i = current; i < newSize; ++i) {
      buf.boidCohesionMemories[i].assign(
          4,
          0.0f); // cohesionMemoriesをmaxNeighbors分確保（dt累積（-1.0fで未使用））
      buf.boidActiveNeighbors[i]
          .reset(); // activeNeighborsをリセット（使用中slotのインデックス）
      buf.boidNeighborIndices[i].fill(-1);
    }
  }

  // ルートが中央バッファを指していることを保証
  if (!root)
    root = getUnitFromPool();
  root->buf = &buf;

  // 再構築
  build(maxBoidsPerUnit);

  // 新しいサイズに合わせて SOA バッファメモリを再初期化
  if (!globalSpeciesParams.empty()) {
    initializeBoidMemories(globalSpeciesParams);
  }

  buf.syncWriteFromRead();
  setRenderPointersToReadBuffers();
}

void BoidTree::collectLeaves(const BoidUnit *node,
                             std::vector<BoidUnit *> &leaves) const {
  if (!node)
    return;
  // children配列の中身がnullptrでないかチェック
  if (node->isBoidUnit()) {
    leaves.push_back(const_cast<BoidUnit *>(node));
  } else {
    for (const auto *child : node->children) {
      if (child)
        collectLeaves(child, leaves);
    }
  }
}

std::unordered_map<int, int> BoidTree::collectBoidUnitMapping() {
  std::unordered_map<int, int> boidUnitMapping;

  std::stack<BoidUnit *> stack;
  stack.push(root);

  while (!stack.empty()) {
    BoidUnit *current = stack.top();
    stack.pop();

    if (current->isBoidUnit()) {
      for (int boidIdx : current->indices) {
        boidUnitMapping[boidIdx] =
            current->id; // BoidのインデックスとユニットIDを対応付け
      }
    } else {
      for (BoidUnit *child : current->children) {
        stack.push(child);
      }
    }
  }

  return boidUnitMapping;
}

uintptr_t BoidTree::getPositionsPtr() {
  if (!renderPositionsPtr_ && !buf.positions.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderPositionsPtr_;
}

uintptr_t BoidTree::getVelocitiesPtr() {
  if (!renderVelocitiesPtr_ && !buf.velocities.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderVelocitiesPtr_;
}
uintptr_t BoidTree::getOrientationsPtr() {
  if (!renderOrientationsPtr_ && !buf.orientations.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderOrientationsPtr_;
}
int BoidTree::getBoidCount() const {
  return static_cast<int>(buf.positions.size());
}

// BoidメモリーとActiveNeighborsを初期化
void BoidTree::initializeBoidMemories(
    const std::vector<SpeciesParams> &speciesParamsList) {
  int totalCount = static_cast<int>(buf.positions.size());

  // 空の speciesParamsList に対する安全装置
  if (speciesParamsList.empty()) {
    logger::log("Warning: speciesParamsList is empty, using default values");

    // バッファのサイズを調整
    buf.boidCohesionMemories.resize(totalCount);
    buf.boidActiveNeighbors.resize(totalCount);
    buf.boidNeighborIndices.resize(totalCount);

    // デフォルト値で初期化
    for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
      buf.boidCohesionMemories[boidIndex].assign(
          4, 0.0f); // デフォルト maxNeighbors = 4
      buf.boidActiveNeighbors[boidIndex].reset();
      buf.boidNeighborIndices[boidIndex].fill(-1);
    }
    return;
  }

  // バッファのサイズを調整
  buf.boidCohesionMemories.resize(totalCount);
  buf.boidActiveNeighbors.resize(totalCount);
  buf.boidNeighborIndices.resize(totalCount);

  // 各Boidごとに実際のspeciesIdに基づいてmaxNeighbors分のメモリを確保
  for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
    int speciesId = buf.speciesIds[boidIndex];

    // speciesIdが有効な範囲内か確認
    if (speciesId < 0 ||
        speciesId >= static_cast<int>(speciesParamsList.size())) {
      // 無効なspeciesIdの場合はデフォルト値を使用
      buf.boidCohesionMemories[boidIndex].assign(4, 0.0f);
      buf.boidActiveNeighbors[boidIndex].reset();
      buf.boidNeighborIndices[boidIndex].fill(-1);
      continue;
    }

    const auto &species = speciesParamsList[speciesId];
    // 近傍スロット数と maxNeighbors がズレると、薄さ判定やFastAttractが常時ONになり
    // 飛び散りやすくなるため、キャッシュ上限でクランプして整合を保つ。
    const int maxNeighbors = glm::clamp(
      species.maxNeighbors, 1,
      static_cast<int>(SoABuffers::NeighborSlotCount));

    // cohesionMemoriesをmaxNeighbors分確保（dt累積（-1.0fで未使用））
    buf.boidCohesionMemories[boidIndex].assign(maxNeighbors, 0.0f);
    // activeNeighborsをリセット（使用中slotのインデックス）
    buf.boidActiveNeighbors[boidIndex].reset();
    buf.boidNeighborIndices[boidIndex].fill(-1);
  }
}

void BoidTree::collectLeavesForCache(BoidUnit *node, BoidUnit *parent) {
  if (!node)
    return;

  if (node->isBoidUnit()) {
    leafCache.push_back(LeafCacheEntry{node, parent});
    return;
  }

  for (auto *child : node->children) {
    if (child)
      collectLeavesForCache(child, node);
  }
}

void BoidTree::setUnitSimpleDensity(int unitId, float value) {
  if (unitId < 0) {
    return;
  }
  const std::size_t index = static_cast<std::size_t>(unitId);
  if (unitSimpleDensities.size() <= index) {
    unitSimpleDensities.resize(index + 1, 0.0f);
  }
  unitSimpleDensities[index] = value;
}

uintptr_t BoidTree::getUnitSimpleDensityPtr() {
  if (unitSimpleDensities.empty()) {
    return 0;
  }
  return reinterpret_cast<uintptr_t>(unitSimpleDensities.data());
}

int BoidTree::getUnitSimpleDensityCount() const {
  return static_cast<int>(unitSimpleDensities.size());
}

namespace {

// 32点程度の近傍グラフを高速にまとめるための簡易Union-Find。
struct SmallUnionFind {
  int parent[32];
  int rank[32];
  int count = 0;

  void init(int n) {
    count = n;
    for (int i = 0; i < n; ++i) {
      parent[i] = i;
      rank[i] = 0;
    }
  }

  int find(int x) {
    int p = parent[x];
    while (p != parent[p]) {
      p = parent[p];
    }
    while (x != parent[x]) {
      int next = parent[x];
      parent[x] = p;
      x = next;
    }
    return p;
  }

  void unite(int a, int b) {
    int ra = find(a);
    int rb = find(b);
    if (ra == rb) {
      return;
    }
    if (rank[ra] < rank[rb]) {
      parent[ra] = rb;
    } else if (rank[ra] > rank[rb]) {
      parent[rb] = ra;
    } else {
      parent[rb] = ra;
      rank[ra] += 1;
    }
  }
};

struct SchoolComponent {
  glm::vec3 sumPos = glm::vec3(0.0f);
  glm::vec3 sumVel = glm::vec3(0.0f);
  float sumWeight = 0.0f;
  float radius = 1.0f;
  int memberCount = 0;
};

} // namespace

void BoidTree::updateSpeciesSchoolClusters(float dt) {
  const std::size_t speciesCount = globalSpeciesParams.size();
  if (speciesCount == 0) {
    speciesSchoolClusters.clear();
    // 中身が変わったので次回JS読み取り時に再パックする
    speciesSchoolClusterBufferDirty = true;
    return;
  }
  if (speciesSchoolClusters.size() != speciesCount) {
    speciesSchoolClusters.assign(speciesCount, {});
  }

  const float safeDt = glm::clamp(dt, 1e-3f, 0.25f);
  const float baseAlpha = glm::clamp(safeDt / kClusterHistorySeconds, 0.01f, 0.2f);

  // species ごとに「小クラスター集合」をまとめて上位クラスタを推定
  for (std::size_t sid = 0; sid < speciesCount; ++sid) {
    auto &schools = speciesSchoolClusters[sid];
    const auto &smallClusters = speciesClusters[sid];
    const SpeciesParams &params = globalSpeciesParams[sid];

    // active な小クラスターをインデックス化（最大 32 前提）
    int activeIndices[32];
    int activeCount = 0;
    for (int i = 0; i < static_cast<int>(smallClusters.size()) && activeCount < 32; ++i) {
      const auto &c = smallClusters[i];
      if (!c.active || c.weight < 0.25f) {
        continue;
      }
      activeIndices[activeCount++] = i;
    }

    // 入力が無い場合は既存のschoolを減衰
    if (activeCount <= 0) {
      for (auto &s : schools) {
        if (!s.active) {
          continue;
        }
        s.weight *= (1.0f - baseAlpha * 1.4f);
        if ((frameCount - s.lastUpdateFrame) > kClusterRetainFrames || s.weight < 0.25f) {
          s.active = false;
        }
      }
      continue;
    }

    // 近接グラフを作り、連結成分（=密集塊）を抽出
    SmallUnionFind uf;
    uf.init(activeCount);

    for (int a = 0; a < activeCount; ++a) {
      const auto &ca = smallClusters[activeIndices[a]];
      const float ra = glm::max(ca.radius, 0.5f);
      for (int b = a + 1; b < activeCount; ++b) {
        const auto &cb = smallClusters[activeIndices[b]];
        const float rb = glm::max(cb.radius, 0.5f);
        const glm::vec3 diff = cb.center - ca.center;
        const float distSq = glm::dot(diff, diff);
        const float link = glm::clamp(kSchoolLinkScale * (ra + rb), 1.0f, 600.0f);
        if (distSq <= link * link) {
          uf.unite(a, b);
        }
      }
    }

    // root -> componentId へ詰め替え（最大32なので配列で十分）
    int rootToComponent[32];
    for (int i = 0; i < 32; ++i) {
      rootToComponent[i] = -1;
    }
    SchoolComponent components[32];
    int componentCount = 0;

    for (int a = 0; a < activeCount; ++a) {
      const int root = uf.find(a);
      int cid = rootToComponent[root];
      if (cid < 0) {
        cid = componentCount++;
        rootToComponent[root] = cid;
        components[cid] = SchoolComponent{};
      }
      const auto &c = smallClusters[activeIndices[a]];
      const float w = glm::clamp(c.weight, 0.25f, 50000.0f);
      components[cid].sumPos += c.center * w;
      components[cid].sumVel += c.avgVelocity * w;
      components[cid].sumWeight += w;
      components[cid].memberCount += 1;
      // radius は後で中心が決まってから再計算したいが、まずは下限を確保
      components[cid].radius = glm::max(components[cid].radius, c.radius);
    }

    // component の中心・半径を確定
    glm::vec3 compCenter[32];
    glm::vec3 compVel[32];
    float compRadius[32];
    float compWeight[32];
    int compMembers[32];

    for (int c = 0; c < componentCount; ++c) {
      const float invW = components[c].sumWeight > 1e-6f ? (1.0f / components[c].sumWeight) : 0.0f;
      compCenter[c] = components[c].sumPos * invW;
      compVel[c] = components[c].sumVel * invW;
      compWeight[c] = components[c].sumWeight;
      compMembers[c] = components[c].memberCount;

      float r = 0.5f;
      // 成分に属する小クラスターから外接半径を取る
      for (int a = 0; a < activeCount; ++a) {
        const int root = uf.find(a);
        const int cid = rootToComponent[root];
        if (cid != c) {
          continue;
        }
        const auto &sc = smallClusters[activeIndices[a]];
        const float dist = glm::length(sc.center - compCenter[c]);
        r = glm::max(r, dist + glm::max(sc.radius, 0.5f));
      }

      // 大クラスターは“群れの中心”として使うので、極端にデカい値は安全側にクランプ
      // （追跡が暴れていた場合でも boid が無限遠の中心を追わないようにする）
      compRadius[c] = glm::clamp(r, 1.0f, glm::max(params.cohesionRange * 6.0f, 60.0f));
    }

    // 既存 school とのマッチング（近い中心へ割り当て）
    bool schoolUsed[64];
    for (int i = 0; i < static_cast<int>(schools.size()) && i < 64; ++i) {
      schoolUsed[i] = false;
    }

    for (int c = 0; c < componentCount; ++c) {
      // ノイズ成分を弾く：単独小クラスターで重みが弱いものは除外
      if (compMembers[c] <= 1 && compWeight[c] < 200.0f) {
        continue;
      }

      int best = -1;
      float bestDistSq = std::numeric_limits<float>::max();

      for (int s = 0; s < static_cast<int>(schools.size()); ++s) {
        auto &school = schools[s];
        if (!school.active || schoolUsed[s]) {
          continue;
        }
        const glm::vec3 diff = compCenter[c] - school.center;
        const float distSq = glm::dot(diff, diff);
        // 半径ベースで「同一群れ」とみなす許容距離を決める
        const float match = glm::clamp((compRadius[c] + school.radius) * 0.85f, 2.0f, 600.0f);
        if (distSq > match * match) {
          continue;
        }
        if (distSq < bestDistSq) {
          bestDistSq = distSq;
          best = s;
        }
      }

      const float hitAlpha = glm::clamp(baseAlpha * (0.35f + 0.10f * static_cast<float>(glm::clamp(compMembers[c], 1, 8))), 0.03f, 0.35f);

      if (best >= 0) {
        auto &school = schools[best];
        schoolUsed[best] = true;
        school.center = glm::mix(school.center, compCenter[c], hitAlpha);
        school.avgVelocity = glm::mix(school.avgVelocity, compVel[c], hitAlpha);
        school.radius = glm::mix(school.radius, compRadius[c], 0.25f);
        school.weight = glm::mix(school.weight, compWeight[c], 0.15f);
        school.lastUpdateFrame = frameCount;
        school.active = true;
      } else {
        // 新規 school 作成 or 弱いものと置換
        BoidTree::SpeciesSchoolCluster school;
        school.center = compCenter[c];
        school.avgVelocity = compVel[c];
        school.radius = compRadius[c];
        school.weight = compWeight[c];
        school.lastUpdateFrame = frameCount;
        school.active = true;

        if (schools.size() < static_cast<std::size_t>(kMaxSchoolsPerSpecies)) {
          schools.push_back(school);
        } else if (!schools.empty()) {
          int weakest = 0;
          float weakestWeight = schools[0].weight;
          for (int s = 1; s < static_cast<int>(schools.size()); ++s) {
            if (schools[s].weight < weakestWeight) {
              weakestWeight = schools[s].weight;
              weakest = s;
            }
          }
          schools[weakest] = school;
        }
      }
    }

    // 更新されなかった school は減衰
    for (auto &s : schools) {
      if (!s.active) {
        continue;
      }
      if ((frameCount - s.lastUpdateFrame) <= 0) {
        continue;
      }
      // 直近でヒットしていない場合は徐々に弱める
      if ((frameCount - s.lastUpdateFrame) > 2) {
        s.weight *= (1.0f - baseAlpha * 1.2f);
      }
      if ((frameCount - s.lastUpdateFrame) > kClusterRetainFrames || s.weight < 0.25f) {
        s.active = false;
      }
    }
  }

  // 中身が更新されたので、次に JS が読むタイミングで再パックする
  speciesSchoolClusterBufferDirty = true;
}

const std::vector<BoidTree::SpeciesSchoolCluster> *
BoidTree::getSpeciesSchoolClusters(int speciesId) const {
  if (speciesId < 0 ||
      speciesId >= static_cast<int>(speciesSchoolClusters.size())) {
    return nullptr;
  }
  return &speciesSchoolClusters[speciesId];
}