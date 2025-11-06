#include "boids_tree.h"

#include "boid_unit.h"
#include "platform_utils.h"
#include "species_params.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// このファイルでは Boid シミュレーションの中枢となる BoidTree
// を実装する。主な責務は 「SoA バッファ管理」「LBVH の構築・更新制御」「Boid
// 間相互作用の計算調整」「運動学的更新」。 毎フレーム update()
// が呼ばれ、移動量に応じて LBVH の refit/rebuild を判定し、
// マルチスレッドで近傍計算→物理更新の順で実行する。

std::vector<SpeciesParams> globalSpeciesParams;

namespace {

// LBVH
// 再構築トリガー用の閾値群。「頻繁すぎる再構築を避けつつ、性能劣化も防ぐ」バランス調整。
constexpr int kMaxFramesWithoutRebuild = 5; // フレーム間隔上限（強制再構築）
constexpr float kMaxAccumulatedMovement = 30.0f; // 累積移動距離の上限
constexpr float kMaxAverageDisplacementPerFrame =
    0.25f; // 平均変位の上限（急激な動きを検知）
constexpr float kMaxSingleFrameDisplacementSq =
    1.0f * 1.0f;                            // 単一 Boid の最大移動²
constexpr float kMovementEpsilonSq = 1e-6f; // 無視できる微小移動の閾値

// LBVH クエリ品質監視用パラメータ。統計から木の劣化を検知し適応的に再構築する。
constexpr float kQualityEwmaAlpha = 0.08f; // EWMA 更新係数（小さいほど安定）
constexpr int kQualityBaselineWarmupQueries = 64; // 基準値確定に必要なクエリ数
constexpr float kQualityNodesThresholdMultiplier = 1.5f; // 許容ノード訪問倍率
constexpr float kQualityBoidsThresholdMultiplier = 2.1f; // 許容 Boid 評価倍率

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

// 内部で LBVH インデックス（葉サイズ 32）を初期化する。
BoidTree::BoidTree() : lbvhIndex_(32) {}

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

// LBVH 空間インデックスを完全再構築し、統計情報をリセットする。
// 大幅な配置変更や初期化時に呼び出され、品質劣化を解消する。
void BoidTree::rebuildSpatialIndex() {
  lbvhIndex_.build(buf);
  framesSinceRebuild_ = 0;
  cumulativeDisplacementSinceRebuild_ = 0.0f;
  maxStepDisplacementSqSinceRebuild_ = 0.0f;
  lastAverageDisplacement_ = 0.0f;
  resetSpatialIndexQuality();
  lbvhDirty_ = false;
}

// 初期構築：最大 Boid 数を設定し、空間インデックスを作成。
// JavaScript 側から最初に呼ばれる初期化エントリーポイント。
void BoidTree::build(int maxPerUnit) {
  maxBoidsPerUnit = maxPerUnit;
  rebuildSpatialIndex();
  setRenderPointersToReadBuffers();
}

// 内部 LBVH インデックスへの直接アクセサー（非 const）。
SpatialIndex &BoidTree::spatialIndex() { return lbvhIndex_; }

// 内部 LBVH インデックスへの読み取り専用アクセサー。
const SpatialIndex &BoidTree::spatialIndex() const { return lbvhIndex_; }

// 全リーフノードを順次訪問するイテレーター。デバッグや統計収集に使用。
void BoidTree::forEachLeaf(const LeafVisitor &visitor) const {
  lbvhIndex_.forEachLeaf(visitor);
}

// 指定球体と交差するリーフノードのみを訪問する空間クエリ。
void BoidTree::forEachLeafIntersectingSphere(const glm::vec3 &center,
                                             float radius,
                                             const LeafVisitor &visitor) const {
  lbvhIndex_.forEachLeafIntersectingSphere(center, radius, visitor);
}

// フレームごとのシミュレーション更新：物理計算・空間インデックス管理・描画準備。
// dt を制限し、Boid 挙動計算、適応的インデックス再構築、統計更新を順次実行。
void BoidTree::update(float dt) {
  const float clampedDt = std::clamp(dt, 0.0f, 0.1f) * 5.0f;

  const int count = static_cast<int>(buf.positions.size());
  if (count == 0 || clampedDt <= 0.0f) {
    setRenderPointersToReadBuffers();
    rebuildSpatialIndex();
    frameCount++;
    return;
  }

  bool rebuiltThisFrame = false;

  // 適応的空間インデックス再構築の判定：
  // フレーム経過数または Boid 移動量が閾値を超えたら完全再構築を実行。
  if (lbvhDirty_) {
    const bool exceededFrameBudget =
        framesSinceRebuild_ >= kMaxFramesWithoutRebuild;
    const bool exceededMotionBudget =
        cumulativeDisplacementSinceRebuild_ >= kMaxAccumulatedMovement ||
        maxStepDisplacementSqSinceRebuild_ >= kMaxSingleFrameDisplacementSq ||
        lastAverageDisplacement_ >= kMaxAverageDisplacementPerFrame;
    if (exceededFrameBudget || exceededMotionBudget) {
      rebuildSpatialIndex();
      rebuiltThisFrame = true;
    }
  }

  // Phase 1: 空間インデックス更新と近傍探索による相互作用力計算
  setRenderPointersToReadBuffers();
  lbvhIndex_.refit(buf); // LBVH バウンディングボックスを現在位置に合わせて更新
  lbvhIndex_.resetQueryStats();
  computeBoidInteractionsRange(buf, lbvhIndex_, 0, count, clampedDt);
  lastQueryStats_ = lbvhIndex_.consumeQueryStats();

  // Phase 2: 物理更新（速度・位置・姿勢）をダブルバッファに書き込み
  setRenderPointersToWriteBuffers();
  updateBoidKinematicsRange(buf, 0, count, clampedDt);
  buf.swapReadWrite(); // 読み書きバッファを入れ替えて結果を確定
  // Phase 3: 移動統計の収集（再構築判定のため）
  setRenderPointersToReadBuffers();
  float frameMaxStepSq = 0.0f;
  float frameSumStepSq = 0.0f;
  if (static_cast<int>(buf.positionsWrite.size()) >= count) {
    for (int i = 0; i < count; ++i) {
      const glm::vec3 delta = buf.positions[i] - buf.positionsWrite[i];
      const float distSq = glm::dot(delta, delta);
      frameMaxStepSq = std::max(frameMaxStepSq, distSq);
      frameSumStepSq += distSq;
    }
  }

  // 平均・最大移動量を算出し、空間インデックス劣化の判定材料とする
  const float invCount = count > 0 ? 1.0f / static_cast<float>(count) : 0.0f;
  const float frameAverageStepSq = frameSumStepSq * invCount;
  const float frameAverageStep =
      frameAverageStepSq > 0.0f ? std::sqrt(frameAverageStepSq) : 0.0f;
  const bool positionsMoved = frameAverageStepSq > kMovementEpsilonSq ||
                              frameMaxStepSq > kMovementEpsilonSq;

  // 累積移動量を更新し、空間インデックスの「汚れ」状態を管理
  if (positionsMoved) {
    cumulativeDisplacementSinceRebuild_ += frameAverageStep;
    maxStepDisplacementSqSinceRebuild_ =
        std::max(maxStepDisplacementSqSinceRebuild_, frameMaxStepSq);
    lbvhDirty_ = true;
  }

  // クエリ品質に基づく追加再構築判定（近傍探索効率の低下を検出）
  const bool qualityWantsRebuild = shouldRebuildForQuality(lastQueryStats_);
  if (!rebuiltThisFrame && qualityWantsRebuild) {
    rebuildSpatialIndex();
    rebuiltThisFrame = true;
  }

  // 統計情報の更新：再構築からの経過フレーム数と移動平均を記録
  if (lbvhDirty_) {
    ++framesSinceRebuild_;
  }
  lastAverageDisplacement_ = frameAverageStep;
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
  rebuildSpatialIndex();
}

// 空間インデックス品質監視統計をリセット：再構築時の初期化。
// EWMA（指数加重移動平均）による性能監視状態をクリアする。
void BoidTree::resetSpatialIndexQuality() {
  queryNodesEwma_ = 0.0f;
  queryBoidsEwma_ = 0.0f;
  queryNodesBaseline_ = 0.0f;
  queryBoidsBaseline_ = 0.0f;
  querySamplesSinceRebuild_ = 0;
  queryBaselineValid_ = false;
}

// クエリ品質統計に基づく再構築必要性判定：探索効率劣化を検出。
// ベースライン比で性能低下が閾値を超えた場合に再構築を推奨する。
bool BoidTree::shouldRebuildForQuality(const LbvhIndex::QueryStats &stats) {
  if (stats.queries <= 0) {
    return false;
  }

  // 現フレームの平均訪問ノード数・検討 Boid 数を算出
  const float invQueries = 1.0f / static_cast<float>(stats.queries);
  const float avgNodes = static_cast<float>(stats.nodesVisited) * invQueries;
  const float avgBoids = static_cast<float>(stats.boidsConsidered) * invQueries;

  // EWMA 更新：初回または既存の移動平均に新データを反映
  if (querySamplesSinceRebuild_ == 0) {
    queryNodesEwma_ = avgNodes;
    queryBoidsEwma_ = avgBoids;
  } else {
    queryNodesEwma_ = kQualityEwmaAlpha * avgNodes +
                      (1.0f - kQualityEwmaAlpha) * queryNodesEwma_;
    queryBoidsEwma_ = kQualityEwmaAlpha * avgBoids +
                      (1.0f - kQualityEwmaAlpha) * queryBoidsEwma_;
  }

  querySamplesSinceRebuild_ += stats.queries;

  // ベースライン確立期間：十分なサンプル数まで待機
  if (!queryBaselineValid_) {
    if (querySamplesSinceRebuild_ >= kQualityBaselineWarmupQueries) {
      queryBaselineValid_ = true;
      queryNodesBaseline_ = std::max(queryNodesEwma_, 1.0f);
      queryBoidsBaseline_ = std::max(queryBoidsEwma_, 1.0f);
    }
    return false;
  }

  // ベースライン更新：現在値が良好なら基準値を下げる（適応的調整）
  queryNodesBaseline_ =
      std::max(std::min(queryNodesBaseline_, queryNodesEwma_), 1.0f);
  queryBoidsBaseline_ =
      std::max(std::min(queryBoidsBaseline_, queryBoidsEwma_), 1.0f);

  // 劣化判定：現在の EWMA がベースライン × 閾値を上回るか
  const bool nodesDegraded =
      queryNodesEwma_ > queryNodesBaseline_ * kQualityNodesThresholdMultiplier;
  const bool boidsDegraded =
      queryBoidsEwma_ > queryBoidsBaseline_ * kQualityBoidsThresholdMultiplier;

  return nodesDegraded || boidsDegraded;
}
