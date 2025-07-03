#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"
#include "species_params.h"
#include <algorithm>
#include <emscripten/val.h>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <mutex>
#include <numeric>
#include <random>
#include <vector>

// グローバル共通
std::vector<SpeciesParams> globalSpeciesParams; // 配列に変更
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
  emscripten::val console = emscripten::val::global("console");
  console.call<void>("log", std::string(params.species));

  auto it = std::find_if(globalSpeciesParams.begin(), globalSpeciesParams.end(),
                         [&params](const SpeciesParams &p) {
                           return p.species == params.species;
                         });
  if (it != globalSpeciesParams.end()) {
    *it = params; // 更新
  } else {
    globalSpeciesParams.push_back(params); // 追加
  }

  // 全種をログに出す
  console.call<void>("log", std::string("Updated species parameters:"));

  for (const auto &sp : globalSpeciesParams) {
    console.call<void>(
        "log", std::string("species: ") + sp.species + ", isPredator: (" +
                   (sp.isPredator ? "true" : "false") +
                   "), maxSpeed: " + std::to_string(sp.maxSpeed));
  }
}

BoidTree::BoidTree()
    : root(nullptr), frameCount(0), splitIndex(0), mergeIndex(0),
      maxBoidsPerUnit(10) {}

// ---- ツリー可視化 ----
void printTree(const BoidUnit *node, int depth) {
  if (!node)
    return;
  emscripten::val console = emscripten::val::global("console");

  std::string indent(depth * 2, ' ');
  // speciesIdを出す
  std::string speciesIdsStr;
  speciesIdsStr = node->speciesId;

  console.call<void>(
      "log", indent + "Level: " + std::to_string(node->level) +
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
    delete root;
  }
  root = new BoidUnit();
  root->buf = &buf; // 中央バッファを共有

  maxBoidsPerUnit = maxPerUnit;
  root->level = level;

  // すべての Boid インデックスを作成
  std::vector<int> indices(buf.positions.size());
  std::iota(indices.begin(), indices.end(), 0);

  buildRecursive(root, indices, maxPerUnit, level);
  // printTree(root, 0);
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
      unit->cohesionMemories.assign(BoidUnit::MAX_BOIDS, 0.0f);
      unit->activeNeighbors.reset();
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

  auto *leftChild = new BoidUnit();
  auto *rightChild = new BoidUnit();
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
      // 種が混在している場合、種ごとに再分割
      std::unordered_map<int, std::vector<int>> speciesGroups;
      for (int i : indices) {
        speciesGroups[buf.speciesIds[i]].push_back(i);
      }

      for (const auto &[speciesId, groupIndices] : speciesGroups) {
        auto *child = new BoidUnit();
        child->buf = node->buf;
        child->parent = node;
        child->topParent = node->topParent ? node->topParent : node;
        child->indices = groupIndices;
        child->speciesId = speciesId;

        for (int gIdx : groupIndices) {
          buf.speciesIds[gIdx] = child->speciesId;
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

  std::vector<int> sorted = indices;
  std::sort(sorted.begin(), sorted.end(), [this, axis](int a, int b) {
    const glm::vec3 &pa = buf.positions[a];
    const glm::vec3 &pb = buf.positions[b];
    return (axis == 0) ? pa.x < pb.x : (axis == 1) ? pa.y < pb.y : pa.z < pb.z;
  });
  std::size_t mid = sorted.size() / 2;
  std::vector<int> left(sorted.begin(), sorted.begin() + mid);
  std::vector<int> right(sorted.begin() + mid, sorted.end());

  auto *leftChild = new BoidUnit();
  auto *rightChild = new BoidUnit();
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
      root->updateRecursive(glm::clamp(dt, 0.0f, 0.1f) * 5);
    } catch (const std::exception &e) {
      std::cerr << "Exception caught: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Unknown exception caught" << std::endl;
    }
  }

  frameCount++;

  // 一定フレームごとに葉ノードを再収集
  if (frameCount % 5 == 0) {
    leafCache.clear();
    //  collectLeaves(root, leafCache); // 葉ノードを収集
    splitIndex = 0;
    mergeIndex = 0;
  }

  // 一定フレームごとに木構造を再構築
  if (frameCount % 2 == 0) { // 再構築頻度を調整
    build(maxBoidsPerUnit, 0);
    // printTree(root, 0); // ツリー構造をログに出力

    return;
  }

  // 分割と結合の処理
  if (!leafCache.empty()) {
    // 分割
    for (int i = 0; i < 12 && splitIndex < (int)leafCache.size();
         ++i, ++splitIndex) {
      emscripten::val console = emscripten::val::global("console");
      console.call<void>("log", std::string("分割"));

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
      emscripten::val console = emscripten::val::global("console");
      console.call<void>("log", std::string("結合"));

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
std::mutex coutMutex;

void BoidTree::initializeBoids(
    const std::vector<SpeciesParams> &speciesParamsList, float posRange,
    float velRange) {
  emscripten::val console = emscripten::val::global("console");
  console.call<void>("log", std::string("speciesParamsList size: ") +
                                std::to_string(speciesParamsList.size()));
  for (const auto &species : speciesParamsList) {
    console.call<void>("log", std::string("Species: ") + species.species +
                                  ", Count: " + std::to_string(species.count));
  }

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

  // バッファを確保
  buf.reserveAll(totalCount);
  buf.positions.resize(totalCount);
  buf.velocities.resize(totalCount);
  buf.accelerations.assign(totalCount, glm::vec3(0.0f));
  buf.speciesIds.resize(totalCount);
  buf.predatorTargetIndices.resize(totalCount);
  buf.predatorTargetTimers.resize(totalCount);

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
      buf.speciesIds[offset] = speciesId;
      ++offset;
    }
  }

  // 木構造を再構築
  if (root)
    delete root;
  root = new BoidUnit();
  root->buf = &buf;

  std::vector<int> indices(totalCount);
  std::iota(indices.begin(), indices.end(), 0);
  buildRecursive(root, indices, maxBoidsPerUnit, 0);
}

// BoidTree::setFlockSize
void BoidTree::setFlockSize(int newSize, float posRange, float velRange) {
  int current = static_cast<int>(buf.positions.size());

  // 個体を減らす
  if (newSize < current) {
    buf.positions.resize(newSize);
    buf.velocities.resize(newSize);
    buf.accelerations.resize(newSize);
    buf.ids.resize(newSize);
    buf.stresses.resize(newSize);
    buf.speciesIds.resize(newSize);
    buf.predatorTargetIndices.resize(newSize, -1);
    buf.predatorTargetTimers.resize(newSize, 0.0f);

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
      buf.positions.push_back(
          glm::vec3(posDist(gen), posDist(gen), posDist(gen)));
      buf.velocities.push_back(
          glm::vec3(velDist(gen), velDist(gen), velDist(gen)));
      buf.accelerations.push_back(glm::vec3(0.0f));
      buf.ids.push_back(i);
      buf.stresses.push_back(0.0f);
      buf.speciesIds.push_back(0);
      buf.cohesionMemories[i] = std::unordered_map<int, float>();
      buf.predatorTargetIndices.push_back(-1);
      buf.predatorTargetTimers.push_back(0.0f);
    }
  }

  // ルートが中央バッファを指していることを保証
  if (!root)
    root = new BoidUnit();
  root->buf = &buf;

  // 再構築
  build(maxBoidsPerUnit, 0);
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
  return reinterpret_cast<uintptr_t>(&buf.positions[0]);
}

uintptr_t BoidTree::getVelocitiesPtr() {
  return reinterpret_cast<uintptr_t>(&buf.velocities[0]);
}
uintptr_t BoidTree::getOrientationsPtr() {
  return reinterpret_cast<uintptr_t>(&buf.orientations[0]);
}
int BoidTree::getBoidCount() const {
  return static_cast<int>(buf.positions.size());
}