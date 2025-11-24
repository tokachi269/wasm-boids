#include "boids_tree.h"

#include "boid_unit.h"
#include "platform_utils.h"
#include "species_params.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <random>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// このファイルでは Boid シミュレーションの中枢となる BoidTree
// を実装する。主な責務は 「SoA バッファ管理」「ユニフォームグリッドを用いた空間インデックス更新」
// 「Boid 間相互作用の計算調整」「運動学的更新」。 毎フレーム update()
// が呼ばれ、グリッドの再構築→近傍計算→物理更新の順で実行する。

std::vector<SpeciesParams> globalSpeciesParams;

namespace {
constexpr int kDbvhActivationOverflowSamples = 4;  // DBVH起動に必要な過負荷検知数
constexpr int kDbvhDeactivateCooldownFrames = 180; // 過負荷解消後の維持フレーム数

// Boid の進行方向から 3D 姿勢（クォータニオン）を計算するヘルパー関数。
// 速度ベクトルを forward 軸とし、world の Y 軸を参考に right/up
// を構成してローカル座標系を作る。 描画時の魚モデル向きに使用される。
glm::quat orientationFromVelocity(const glm::vec3 &velocity) {
  const float speedSq = glm::dot(velocity, velocity);
  if (speedSq < 1e-8f) {
    return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // 停止時はデフォルト姿勢
  }

  const glm::vec3 forward = velocity * (1.0f / glm::sqrt(speedSq));
  glm::vec3 up(0.0f, 1.0f, 0.0f);
  if (std::fabs(glm::dot(forward, up)) > 0.99f) {
    up = glm::vec3(1.0f, 0.0f, 0.0f); // ほぼ垂直なら X 軸を up に変更
  }

  glm::vec3 right = glm::cross(up, forward);
  const float rightLenSq = glm::dot(right, right);
  if (rightLenSq < 1e-8f) {
    right = glm::vec3(1.0f, 0.0f, 0.0f); // 外積がゼロなら X 軸を right に
  } else {
    right *= 1.0f / glm::sqrt(rightLenSq); // 正規化
  }
  const glm::vec3 correctedUp = glm::cross(forward, right); // 再直交化
  const glm::mat3 basis(right, correctedUp, forward);       // 座標系行列
  return glm::quat_cast(basis); // クォータニオンに変換
}

} // namespace

// シングルトンパターンで BoidTree のグローバルインスタンスを提供。
// アプリケーション全体で一つの Boid 管理器を共有する設計。
BoidTree &BoidTree::instance() {
  static BoidTree instance;
  return instance;
}

// 内部でユニフォームグリッドインデックスを初期化する。
BoidTree::BoidTree() = default;

BoidTree::~BoidTree() = default;

// JavaScript 側描画用に SoA バッファのアドレスを準備する。
// ダブルバッファリング：計算は soaBuffers_、描画は buf を使い分け。
void BoidTree::setRenderPointersToReadBuffers() {
  renderPositionsPtr_ = buf.positions.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(buf.positions.data());
  renderVelocitiesPtr_ =
      buf.velocities.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.velocities.data());
  renderOrientationsPtr_ =
      buf.orientations.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.orientations.data());
}

// JavaScript 側描画用に書き込み用バッファのアドレスを準備する。
// 計算中でも描画が継続できるよう、Write バッファを描画ターゲットに設定。
void BoidTree::setRenderPointersToWriteBuffers() {
  renderPositionsPtr_ =
      buf.positionsWrite.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.positionsWrite.data());
  renderVelocitiesPtr_ =
      buf.velocitiesWrite.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.velocitiesWrite.data());
  renderOrientationsPtr_ =
      buf.orientationsWrite.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.orientationsWrite.data());
}

// ユニフォームグリッドを再構築し、近傍探索用のセル割り当てを更新する。
// 大幅な配置変更や初期化時に呼び出され、最新位置に基づくセル分布を作り直す。
void BoidTree::rebuildSpatialIndex() {
  gridIndex_.setCellSize(gridCellSize_);
  gridIndex_.build(buf);
  if (dbvhEnabled_) {
    const float baseRadius = std::max(gridCellSize_ * 0.5f, 1.0f);
    const float velocityPadding = baseRadius * dbvhPaddingScale_;
    dbvhIndex_.sync(buf, baseRadius, velocityPadding);
  } else if (!dbvhIndex_.empty()) {
    dbvhIndex_.clear();
  }
}

// 種族パラメータを参考にセルサイズを適応調整する。
// 最大行動半径に少し余裕を持たせ、最小セル幅を確保する。
void BoidTree::updateGridCellSize() {
  float target = gridCellSize_;
  float maxRange = 0.0f;
  for (const auto &params : globalSpeciesParams) {
    float speciesRange = std::max(params.cohesionRange, params.alignmentRange);
    speciesRange = std::max(speciesRange, params.separationRange);
    speciesRange = std::max(speciesRange, params.predatorAlertRadius);
    maxRange = std::max(maxRange, speciesRange);
  }

  if (maxRange <= 0.0f) {
    target = std::max(10.0f, gridCellSize_);
  } else {
    target = std::max(10.0f, maxRange * 1.1f);
  }
  target = 3.0f;
  if (std::fabs(target - gridCellSize_) > 1e-3f) {
    gridCellSize_ = target;
    gridIndex_.setCellSize(gridCellSize_);
  }
}

GridDbvhNeighborProvider
BoidTree::makeNeighborProvider(HybridNeighborStats *stats) const {
  const bool fallbackReady = dbvhEnabled_ && !dbvhIndex_.empty();
  const DbvhIndex *fallbackIndex = fallbackReady ? &dbvhIndex_ : nullptr;
  return GridDbvhNeighborProvider(buf, gridIndex_, fallbackIndex,
                                  neighborFallbackThreshold_,
                                  neighborFallbackLimit_,
                                  neighborFallbackRadiusScale_, stats);
}

void BoidTree::updateDbvhState(const HybridNeighborStats &stats) {
  if (stats.overflowCount >= kDbvhActivationOverflowSamples) {
    dbvhEnabled_ = true;
    dbvhDisableCooldown_ = kDbvhDeactivateCooldownFrames;
    return;
  }

  if (!dbvhEnabled_) {
    return;
  }

  if (stats.overflowCount > 0 || stats.fallbackQueries > 0) {
    dbvhDisableCooldown_ = kDbvhDeactivateCooldownFrames;
    return;
  }

  if (dbvhDisableCooldown_ > 0) {
    dbvhDisableCooldown_--;
  } else {
    dbvhEnabled_ = false;
  }
}

// 初期構築：最大 Boid 数を設定し、空間インデックスを作成。
// JavaScript 側から最初に呼ばれる初期化エントリーポイント。
void BoidTree::build(int maxPerUnit) {
  maxBoidsPerUnit = maxPerUnit;
  updateGridCellSize();
  rebuildSpatialIndex();
  setRenderPointersToReadBuffers();
}

// 内部ユニフォームグリッドへの直接アクセサー（非 const）。
SpatialIndex &BoidTree::spatialIndex() { return gridIndex_; }

// 内部ユニフォームグリッドへの読み取り専用アクセサー。
const SpatialIndex &BoidTree::spatialIndex() const { return gridIndex_; }

// 全リーフノードを順次訪問するイテレーター。デバッグや統計収集に使用。
void BoidTree::forEachLeaf(const LeafVisitor &visitor) const {
  gridIndex_.forEachLeaf(visitor);
}

// 指定球体と交差するリーフノードのみを訪問する空間クエリ。
void BoidTree::forEachLeafIntersectingSphere(const glm::vec3 &center,
                                             float radius,
                                             const LeafVisitor &visitor) const {
  gridIndex_.forEachLeafIntersectingSphere(center, radius, visitor);
}

// フレームごとのシミュレーション更新：物理計算・空間インデックス管理・描画準備。
// dt を制限し、近傍力計算→運動学更新→グリッド再構築の順で処理する。
void BoidTree::update(float dt) {
  const float clampedDt = std::clamp(dt, 0.0f, 0.1f) * 5.0f;

  const int count = static_cast<int>(buf.positions.size());
  if (count == 0 || clampedDt <= 0.0f) {
    setRenderPointersToReadBuffers();
    updateGridCellSize();
    rebuildSpatialIndex();
    frameCount++;
    return;
  }

  setRenderPointersToReadBuffers();
  gridIndex_.setSamplingSeed(static_cast<std::uint32_t>(frameCount));
  HybridNeighborStats neighborStats{};
  const GridDbvhNeighborProvider neighborProvider =
      makeNeighborProvider(&neighborStats);
  computeBoidInteractionsRange(buf, neighborProvider, 0, count, clampedDt);

  setRenderPointersToWriteBuffers();
  updateBoidKinematicsRange(buf, 0, count, clampedDt);
  buf.swapReadWrite(); // 読み書きバッファを入れ替えて結果を確定
  setRenderPointersToReadBuffers();

  // 次フレーム用にセルサイズを更新し、最新位置でグリッドを再構築
  updateDbvhState(neighborStats);
  updateGridCellSize();
  rebuildSpatialIndex();
  frameCount++;
}

// Boid の初期配置：種族パラメータに基づいて個体を生成・配置する。
// 位置・速度・色をランダム分布で設定し、SoA バッファに格納。
void BoidTree::initializeBoids(
    const std::vector<SpeciesParams> &speciesParamsList, float posRange,
    float velRange) {
  globalSpeciesParams = speciesParamsList;

  int totalCount = 0;
  for (const auto &species : globalSpeciesParams) {
    totalCount += std::max(species.count, 0);
  }

  // SoA バッファの容量確保とサイズ設定
  buf.reserveAll(totalCount);
  buf.resizeAll(totalCount);

  // 全フィールドをゼロクリア：初期状態では外力・ストレス・捕食関係なし
  std::fill(buf.accelerations.begin(), buf.accelerations.end(),
            glm::vec3(0.0f));
  std::fill(buf.predatorInfluences.begin(), buf.predatorInfluences.end(),
            glm::vec3(0.0f));
  std::fill(buf.predatorThreats.begin(), buf.predatorThreats.end(), 0.0f);
  std::fill(buf.predatorTargetIndices.begin(), buf.predatorTargetIndices.end(),
            -1);
  std::fill(buf.predatorTargetTimers.begin(), buf.predatorTargetTimers.end(),
            0.0f);
  std::fill(buf.stresses.begin(), buf.stresses.end(), 0.0f);
  std::fill(buf.isAttracting.begin(), buf.isAttracting.end(), 0);
  std::fill(buf.attractTimers.begin(), buf.attractTimers.end(), 0.0f);

  // ランダム生成器と分布の準備：位置・速度を指定範囲内で均等分布
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> posDist(-posRange, posRange);
  std::uniform_real_distribution<float> velDist(-velRange, velRange);

  // 種族ごとに個体を生成：各種族の count 数だけ Boid を配置
  int offset = 0;
  for (std::size_t speciesId = 0; speciesId < globalSpeciesParams.size();
       ++speciesId) {
    const auto &species = globalSpeciesParams[speciesId];
    const int countForSpecies = std::max(species.count, 0);
    for (int i = 0; i < countForSpecies; ++i, ++offset) {
      const glm::vec3 position(posDist(gen), posDist(gen), posDist(gen));
      const glm::vec3 velocity(velDist(gen), velDist(gen), velDist(gen));
      const glm::quat orientation = orientationFromVelocity(velocity);

      // 各 Boid の基本属性を SoA バッファに設定
      buf.positions[offset] = position;
      buf.velocities[offset] = velocity;
      buf.ids[offset] = offset;
      buf.speciesIds[offset] = static_cast<int>(speciesId);
      buf.orientations[offset] = orientation;
    }
  }

  // 初期化完了処理：ダブルバッファ同期、描画準備、空間インデックス構築
  buf.syncWriteFromRead();
  setRenderPointersToReadBuffers();
  initializeBoidMemories(globalSpeciesParams);
  updateGridCellSize();
  rebuildSpatialIndex();
  frameCount = 0;
}

// Boid の近傍記憶領域を初期化：各個体が記憶する最大近傍数を設定。
// メモリ効率のため、種族ごとの maxNeighborCount に基づいて動的確保。
void BoidTree::initializeBoidMemories(
    const std::vector<SpeciesParams> &speciesParamsList) {
  const int totalCount = static_cast<int>(buf.positions.size());

  // Boid が存在しない場合は記憶領域をクリア
  if (totalCount == 0) {
    buf.boidCohesionMemories.clear();
    buf.boidNeighborMasks.clear();
    buf.boidNeighborIndices.clear();
    return;
  }

  // 近傍記憶用バッファのサイズ確保：結束記憶・マスク・インデックス配列
  buf.boidCohesionMemories.resize(totalCount);
  buf.boidNeighborMasks.resize(totalCount, 0);
  buf.boidNeighborIndices.resize(totalCount);

  // 種族パラメータが未設定時のフォールバック（デフォルト値使用）
  if (speciesParamsList.empty()) {
    logger::log(
        "Warning: speciesParamsList is empty, using default neighbor slots");
    for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
      buf.boidCohesionMemories[boidIndex].assign(4, 0.0f);
      buf.boidNeighborMasks[boidIndex] = 0;
      buf.boidNeighborIndices[boidIndex].fill(-1);
    }
    return;
  }

  // 個体ごとに種族パラメータに基づく近傍記憶スロットを確保
  for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
    const int speciesId = (boidIndex < static_cast<int>(buf.speciesIds.size()))
                              ? buf.speciesIds[boidIndex]
                              : -1;
    // 無効な種族 ID の場合はデフォルト設定を適用
    if (speciesId < 0 ||
        speciesId >= static_cast<int>(speciesParamsList.size())) {
      buf.boidCohesionMemories[boidIndex].assign(4, 0.0f);
      buf.boidNeighborMasks[boidIndex] = 0;
      buf.boidNeighborIndices[boidIndex].fill(-1);
      continue;
    }

    // 種族の maxNeighbors に従って記憶スロット数を決定（1〜16 に制限）
    const auto &species = speciesParamsList[speciesId];
    const int slotCount = std::clamp(species.maxNeighbors, 1, 16);
    buf.boidCohesionMemories[boidIndex].assign(slotCount, 0.0f);
    buf.boidNeighborMasks[boidIndex] = 0;
    buf.boidNeighborIndices[boidIndex].fill(-1);
  }
}

// Boid 群のサイズを動的に変更：新規追加・削除・リサイズに対応。
// 既存データを保持しつつ、不足分はランダム生成で補完する。
void BoidTree::setFlockSize(int newSize, float posRange, float velRange) {
  newSize = std::max(0, newSize);
  const int current = static_cast<int>(buf.positions.size());
  if (newSize == current) {
    return;
  }

  // サイズ縮小：既存 Boid を削除
  if (newSize < current) {
    buf.positions.resize(newSize);
    buf.positionsWrite.resize(newSize);
    buf.velocities.resize(newSize);
    buf.velocitiesWrite.resize(newSize);
    buf.accelerations.resize(newSize);
    buf.ids.resize(newSize);
    buf.stresses.resize(newSize);
    buf.speciesIds.resize(newSize);
    buf.isAttracting.resize(newSize);
    buf.attractTimers.resize(newSize);
    buf.orientations.resize(newSize);
    buf.orientationsWrite.resize(newSize);
    buf.predatorTargetIndices.resize(newSize, -1);
    buf.predatorTargetTimers.resize(newSize, 0.0f);
    buf.predatorInfluences.resize(newSize);
    buf.predatorThreats.resize(newSize);
    buf.boidCohesionMemories.resize(newSize);
    buf.boidNeighborMasks.resize(newSize, 0);
    buf.boidNeighborIndices.resize(newSize);
    // 削除された Boid の結束記憶をマップから除去
    for (int i = newSize; i < current; ++i) {
      buf.cohesionMemories.erase(i);
    }
  } else {
    // サイズ拡大：新規 Boid をランダム生成して追加
    const int addCount = newSize - current;
    buf.reserveAll(newSize);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int k = 0; k < addCount; ++k) {
      const int index = current + k;
      const glm::vec3 position(posDist(gen), posDist(gen), posDist(gen));
      const glm::vec3 velocity(velDist(gen), velDist(gen), velDist(gen));
      const glm::quat orientation = orientationFromVelocity(velocity);

      // 新規 Boid の全フィールドを初期値で SoA バッファに追加
      buf.positions.push_back(position);
      buf.positionsWrite.push_back(position);
      buf.velocities.push_back(velocity);
      buf.velocitiesWrite.push_back(velocity);
      buf.accelerations.push_back(glm::vec3(0.0f));
      buf.ids.push_back(index);
      buf.stresses.push_back(0.0f);
      buf.speciesIds.push_back(0); // デフォルト種族 ID
      buf.isAttracting.push_back(0);
      buf.attractTimers.push_back(0.0f);
      buf.orientations.push_back(orientation);
      buf.orientationsWrite.push_back(orientation);
      buf.predatorTargetIndices.push_back(-1);
      buf.predatorTargetTimers.push_back(0.0f);
      buf.predatorInfluences.push_back(glm::vec3(0.0f));
      buf.predatorThreats.push_back(0.0f);
      buf.boidCohesionMemories.emplace_back();
      buf.boidNeighborMasks.push_back(0);
      buf.boidNeighborIndices.emplace_back();
      buf.boidNeighborIndices.back().fill(-1);
      buf.cohesionMemories[index] = std::unordered_map<int, float>();
    }
  }

  // サイズ変更後の初期化：記憶領域再構築、バッファ同期、空間インデックス更新
  initializeBoidMemories(globalSpeciesParams);
  buf.syncWriteFromRead();
  setRenderPointersToReadBuffers();
  updateGridCellSize();
  rebuildSpatialIndex();
}

// JavaScript 側描画用の位置データポインタを取得。
// 未設定時は自動的に読み取りバッファを指すよう設定。
uintptr_t BoidTree::getPositionsPtr() {
  if (!renderPositionsPtr_ && !buf.positions.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderPositionsPtr_;
}

// JavaScript 側描画用の速度データポインタを取得。
// Three.js でのベクトル矢印描画などに使用される。
uintptr_t BoidTree::getVelocitiesPtr() {
  if (!renderVelocitiesPtr_ && !buf.velocities.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderVelocitiesPtr_;
}

// JavaScript 側描画用の姿勢データポインタを取得。
// Boid の向きに基づく 3D モデル回転に使用。
uintptr_t BoidTree::getOrientationsPtr() {
  if (!renderOrientationsPtr_ && !buf.orientations.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderOrientationsPtr_;
}

// 現在の Boid 総数を取得（配列サイズベース）。
int BoidTree::getBoidCount() const {
  return static_cast<int>(buf.positions.size());
}

// Boid インデックスマッピングを収集：現在は恒等写像。
// 将来的に Boid ID とバッファインデックスが異なる場合に対応。
std::unordered_map<int, int> BoidTree::collectBoidUnitMapping() {
  std::unordered_map<int, int> mapping;
  const int count = static_cast<int>(buf.positions.size());
  mapping.reserve(count);
  for (int i = 0; i < count; ++i) {
    mapping.emplace(i, i);
  }
  return mapping;
}

// 指定名の種族パラメータを検索・取得する。
// JavaScript 側から種族設定を取得する際のインターフェース。
SpeciesParams BoidTree::getGlobalSpeciesParams(std::string species) {
  const auto it = std::find_if(
      globalSpeciesParams.begin(), globalSpeciesParams.end(),
      [&species](const SpeciesParams &p) { return p.species == species; });
  if (it != globalSpeciesParams.end()) {
    return *it;
  }
  return {};
}

// 種族パラメータを設定・更新：既存種族は上書き、新規種族は追加。
// パラメータ変更後は記憶領域と空間インデックスを再初期化する。
void BoidTree::setGlobalSpeciesParams(const SpeciesParams &params) {
  const auto it =
      std::find_if(globalSpeciesParams.begin(), globalSpeciesParams.end(),
                   [&params](const SpeciesParams &p) {
                     return p.species == params.species;
                   });
  if (it != globalSpeciesParams.end()) {
    *it = params;
  } else {
    globalSpeciesParams.push_back(params);
  }

  initializeBoidMemories(globalSpeciesParams);
  updateGridCellSize();
  rebuildSpatialIndex();
}

float BoidTree::computeAverageNeighborRadius() const {
  if (globalSpeciesParams.empty()) {
    return gridCellSize_;
  }

  float weightedRangeSum = 0.0f;
  int totalCount = 0;
  for (const auto &params : globalSpeciesParams) {
    const float range = std::max(
        std::max(params.cohesionRange, params.alignmentRange),
        std::max(params.separationRange, params.predatorAlertRadius));
    const int weight = std::max(params.count, 0);
    weightedRangeSum += range * static_cast<float>(weight);
    totalCount += weight;
  }

  if (totalCount <= 0) {
    return gridCellSize_;
  }

  return weightedRangeSum / static_cast<float>(totalCount);
}
