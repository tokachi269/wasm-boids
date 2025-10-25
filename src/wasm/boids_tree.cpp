#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"
#include "platform_utils.h"
#include "species_params.h"
#include <algorithm>
#include <cfloat>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>


// グローバル共通
std::vector<SpeciesParams> globalSpeciesParams;
// 静的メンバー変数の初期化
int BoidUnit::nextId = 0;

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

  // パラメータ変更後、per-Boid メモリリストを再構築
  initializeBoidMemories(globalSpeciesParams);
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
    unit->parent = nullptr;
    unit->topParent = nullptr;
    unit->children.clear();
    unit->indices.clear();
    unit->center = glm::vec3(0.0f);
    unit->averageVelocity = glm::vec3(0.0f);
    unit->radius = 0.0f;
    unit->level = 0;
    unit->frameCount = 0;
    unit->speciesId = -1;
    unit->buf = &buf;

    return unit;
  }

  // プールが空の場合は新規作成
  BoidUnit *unit = new BoidUnit();
  unit->buf = &buf;
  return unit;
}

void BoidTree::returnUnitToPool(BoidUnit *unit) {
  if (!unit)
    return;

  // 再帰的に子ノードも返却
  for (BoidUnit *child : unit->children) {
    returnUnitToPool(child);
  }
  unit->children.clear();

  // プールに返却
  unitPool.push(unit);
}

void BoidTree::returnNodeToPool(BoidUnit *node) {
  if (!node)
    return;

  // 再帰的に子ノードも返却
  for (BoidUnit *child : node->children) {
    returnNodeToPool(child);
  }
  node->children.clear();

  // プールに返却
  unitPool.push(node);
}

void BoidTree::clearPool() {
  while (!unitPool.empty()) {
    delete unitPool.top();
    unitPool.pop();
  }
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

// ---- ツリー可視化 ----
void printTree(const BoidUnit *node, int depth) {
  if (!node)
    return;

  std::string indent(depth * 2, ' ');
  // speciesIdを出す
  std::string speciesIdsStr;
  speciesIdsStr = node->speciesId;

  logger::log(indent + "Level: " + std::to_string(node->level) +
              " | Boids: " + std::to_string(node->indices.size()) +
              " | Children: " + std::to_string(node->children.size()) +
              " | Center: (" + std::to_string(node->center.x) + ", " +
              std::to_string(node->center.y) + ", " +
              std::to_string(node->center.z) + ")" +
              " | Radius: " + std::to_string(node->radius) +
              " | speciesIds: [" + std::to_string(node->speciesId) + "]");

  for (const auto *child : node->children)
    printTree(child, depth + 1);
}
void BoidTree::build(int maxPerUnit, int level) {
  // 既存の root を削除して再生成
  if (root) {
    returnNodeToPool(root);
  }
  root = getUnitFromPool();
  root->buf = &buf; // 中央バッファを共有

  maxBoidsPerUnit = maxPerUnit;
  root->level = level;

  // すべての Boid インデックスを作成
  std::vector<int> indices(buf.positions.size());
  std::iota(indices.begin(), indices.end(), 0);

  buildRecursive(root, indices, maxPerUnit, level);
  // printTree(root, 0);
  setRenderPointersToReadBuffers();
}
// void BoidTree::build(int maxPerUnit, int level) {
//   // rootが存在する場合は再利用して再構築
//   if (root) {
//     std::vector<BoidUnit *> existingUnits;
//     collectLeaves(root, existingUnits); // 既存の葉ノードを収集

//     // 新しいルートノードを作成
//     auto *newRoot = new BoidUnit();
//     newRoot->buf = &buf; // 中央バッファを共有
//     newRoot->level = level;

//     // 既存のユニットを再利用して再構築
//     rebuildTreeWithUnits(newRoot, existingUnits, maxPerUnit, level);

//     // 古いルートを削除
//     delete root;
//     root = newRoot;
//   } else {
//     // rootが存在しない場合は新規作成
//     root = new BoidUnit();
//     root->buf = &buf; // 中央バッファを共有
//     maxBoidsPerUnit = maxPerUnit;
//     root->level = level;

//     // すべての Boid インデックスを作成
//     std::vector<int> indices(buf.positions.size());
//     std::iota(indices.begin(), indices.end(), 0);

//     buildRecursive(root, indices, maxPerUnit, level);
//   }
// }

// 既存のユニットを再利用して木構造を再構築する関数
void BoidTree::rebuildTreeWithUnits(BoidUnit *node,
                                    const std::vector<BoidUnit *> &units,
                                    int maxPerUnit, int level) {
  node->level = level;

  // 既存の children をクリア
  node->children.clear();

  // 空間的に分割
  if ((int)units.size() <= maxPerUnit) {
    // 葉ノードとしてユニットを直接保持
    for (auto *unit : units) {
      unit->parent = node; // 親ノードを設定
      unit->topParent =
          node->topParent ? node->topParent : node; // トップノードを継承

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
  leftChild->parent = node;
  rightChild->parent = node;
  leftChild->topParent = node->topParent ? node->topParent : node;
  rightChild->topParent = node->topParent ? node->topParent : node;

  node->children.push_back(leftChild);
  node->children.push_back(rightChild);

  rebuildTreeWithUnits(leftChild, left, maxPerUnit, level + 1);
  rebuildTreeWithUnits(rightChild, right, maxPerUnit, level + 1);

  node->computeBoundingSphere(); // バウンディングスフィアを計算
}

void BoidTree::buildRecursive(BoidUnit *node, const std::vector<int> &indices,
                              int maxPerUnit, int level) {
  node->level = level;

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
        child->parent = node;
        child->topParent = node->topParent ? node->topParent : node;
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
  leftChild->parent = node;
  rightChild->parent = node;
  leftChild->topParent = node->topParent ? node->topParent : node;
  rightChild->topParent = node->topParent ? node->topParent : node;

  node->children.push_back(leftChild);
  node->children.push_back(rightChild);

  buildRecursive(leftChild, left, maxPerUnit, level + 1);
  buildRecursive(rightChild, right, maxPerUnit, level + 1);

  node->speciesId = -1; // 内部ノードは種を持たない
  node->computeBoundingSphere();
}

BoidUnit *BoidTree::findParent(BoidUnit *node, BoidUnit *target) {
  if (!node || node->children.empty())
    return nullptr;
  for (auto *c : node->children) {
    if (!c)
      continue;
    if (c == target)
      return node;
    BoidUnit *res = findParent(c, target);
    if (res)
      return res;
  }
  return nullptr;
}

void BoidTree::update(float dt) {
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

  frameCount++;

  // 一定フレームごとに葉ノードを再収集
  if (frameCount % 15 == 0) { // 15フレーム（約0.25秒）ごとに収集
    leafCache.clear();
    //  collectLeaves(root, leafCache); // 葉ノードを収集
    splitIndex = 0;
    mergeIndex = 0;
  }

  // 一定フレームごとに木構造を再構築（大幅に頻度を減らす）
  if (frameCount % 10 == 0) { // 10フレーム（約0.1秒）ごとに再構築
    build(maxBoidsPerUnit, 0);
    // printTree(root, 0); // ツリー構造をログに出力

    // return;
  }

  // 分割と結合の処理
  if (!leafCache.empty()) {
    // 分割
    for (int i = 0; i < 12 && splitIndex < (int)leafCache.size();
         ++i, ++splitIndex) {
      BoidUnit *u = leafCache[splitIndex];
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
        BoidUnit *a = leafCache[mergeIndex];
        BoidUnit *b = leafCache[j];
        if (a && b && a->canMergeWith(*b)) {
          BoidUnit *parent = findParent(root, b);
          if (parent) {
            leafCache.clear();
            a->mergeWith(b, parent);
          }
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
    for (int i = 0; i < species.count; ++i) {
      buf.positions[offset] =
          glm::vec3(posDist(gen), posDist(gen), posDist(gen));
      buf.velocities[offset] =
          glm::vec3(velDist(gen), velDist(gen), velDist(gen));
      buf.ids[offset] = offset;
      buf.speciesIds[offset] = speciesId;
      ++offset;
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
  buildRecursive(root, indices, maxBoidsPerUnit, 0);

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
    buf.boidCohesionMemories.resize(newSize);
    buf.boidActiveNeighbors.resize(newSize);

    // cohesionMemories は erase で縮小
    for (int i = newSize; i < current; ++i) {
      buf.cohesionMemories.erase(i);
    }
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
      buf.cohesionMemories[i] = std::unordered_map<int, float>();
      buf.predatorTargetIndices.push_back(-1);
      buf.predatorTargetTimers.push_back(0.0f);
    }

    // 新しいBoidのメモリを初期化
    buf.boidCohesionMemories.resize(newSize);
    buf.boidActiveNeighbors.resize(newSize);

    // 新しく追加されたBoidのメモリを初期化（デフォルトのmaxNeighbors=4を使用）
    for (int i = current; i < newSize; ++i) {
      buf.boidCohesionMemories[i].assign(
          4,
          0.0f); // cohesionMemoriesをmaxNeighbors分確保（dt累積（-1.0fで未使用））
      buf.boidActiveNeighbors[i]
          .reset(); // activeNeighborsをリセット（使用中slotのインデックス）
    }
  }

  // ルートが中央バッファを指していることを保証
  if (!root)
    root = getUnitFromPool();
  root->buf = &buf;

  // 再構築
  build(maxBoidsPerUnit, 0);

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

    // デフォルト値で初期化
    for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
      buf.boidCohesionMemories[boidIndex].assign(
          4, 0.0f); // デフォルト maxNeighbors = 4
      buf.boidActiveNeighbors[boidIndex].reset();
    }
    return;
  }

  // バッファのサイズを調整
  buf.boidCohesionMemories.resize(totalCount);
  buf.boidActiveNeighbors.resize(totalCount);

  // 各Boidごとに実際のspeciesIdに基づいてmaxNeighbors分のメモリを確保
  for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
    int speciesId = buf.speciesIds[boidIndex];

    // speciesIdが有効な範囲内か確認
    if (speciesId < 0 ||
        speciesId >= static_cast<int>(speciesParamsList.size())) {
      // 無効なspeciesIdの場合はデフォルト値を使用
      buf.boidCohesionMemories[boidIndex].assign(4, 0.0f);
      buf.boidActiveNeighbors[boidIndex].reset();
      continue;
    }

    const auto &species = speciesParamsList[speciesId];
    int maxNeighbors = std::max(1, species.maxNeighbors); // 最小 1個 保証

    // cohesionMemoriesをmaxNeighbors分確保（dt累積（-1.0fで未使用））
    buf.boidCohesionMemories[boidIndex].assign(maxNeighbors, 0.0f);
    // activeNeighborsをリセット（使用中slotのインデックス）
    buf.boidActiveNeighbors[boidIndex].reset();
  }
}