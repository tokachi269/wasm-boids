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

// ---- BoidTree::build ----
void BoidTree::build(int maxPerUnit, int level) {
  if (!root) {
    root = new BoidUnit();
    root->buf = &buf; // 中央バッファを共有
  }
  maxBoidsPerUnit = maxPerUnit;
  root->level = level;

  // すべての Boid インデックスを作成
  std::vector<int> indices(buf.positions.size());
  std::iota(indices.begin(), indices.end(), 0);

  buildRecursive(root, indices, maxPerUnit, level);
  
  //   emscripten::val console = emscripten::val::global("console");
  // // console.call<void>("log", std::string("printTree called."));
  // // printTree(root, 0);
}

void BoidTree::buildRecursive(BoidUnit *node, const std::vector<int> &indices,
                              int maxPerUnit, int level) {
    node->level = level;

    // 末端ノード（葉）の処理
    if ((int)indices.size() <= maxPerUnit) {
        // ユニット内の speciesId を確認
        int firstSpeciesId = buf.speciesIds[indices[0]];
        for (int i : indices) {
            if (buf.speciesIds[i] != firstSpeciesId) {
                throw std::runtime_error("Mixed speciesId in a single unit");
            }
        }

        node->indices = indices;          // インデックスだけコピー
        node->speciesId = firstSpeciesId; // speciesId を登録

        // 再設定: buf->speciesIds
        for (int gIdx : indices) {
            buf.speciesIds[gIdx] = node->speciesId;
        }

        node->computeBoundingSphere();
        return;
    }

    // speciesId ごとにインデックスを分割
    std::unordered_map<int, std::vector<int>> speciesGroups;
    for (int i : indices) {
        speciesGroups[buf.speciesIds[i]].push_back(i);
    }

    // speciesId ごとに再帰処理
    for (const auto &[speciesId, groupIndices] : speciesGroups) {
        if (groupIndices.size() <= maxPerUnit) {
            // 新しいユニットを作成
            auto *child = new BoidUnit();
            child->buf = node->buf;
            child->indices = groupIndices;
            child->speciesId = speciesId;

            // 再設定: buf->speciesIds
            for (int gIdx : groupIndices) {
                buf.speciesIds[gIdx] = child->speciesId;
            }

            child->computeBoundingSphere();
            node->children.push_back(child);
        } else {
            // 通常の空間分割処理
            float mean[3] = {0}, var[3] = {0};
            for (int i : groupIndices) {
                const glm::vec3 &p = buf.positions[i];
                mean[0] += p.x;
                mean[1] += p.y;
                mean[2] += p.z;
            }
            for (int k = 0; k < 3; ++k)
                mean[k] /= groupIndices.size();

            for (int i : groupIndices) {
                const glm::vec3 &p = buf.positions[i];
                var[0] += (p.x - mean[0]) * (p.x - mean[0]);
                var[1] += (p.y - mean[1]) * (p.y - mean[1]);
                var[2] += (p.z - mean[2]) * (p.z - mean[2]);
            }
            int axis = (var[1] > var[0]) ? 1 : 0;
            if (var[2] > var[axis])
                axis = 2;

            // インデックスをソートして 2 つに分割
            std::vector<int> sorted = groupIndices;
            std::sort(sorted.begin(), sorted.end(), [this, axis](int a, int b) {
                const glm::vec3 &pa = buf.positions[a];
                const glm::vec3 &pb = buf.positions[b];
                return (axis == 0)   ? pa.x < pb.x
                       : (axis == 1) ? pa.y < pb.y
                                     : pa.z < pb.z;
            });
            std::size_t mid = sorted.size() / 2;
            std::vector<int> left(sorted.begin(), sorted.begin() + mid);
            std::vector<int> right(sorted.begin() + mid, sorted.end());

            // 子ノードを生成して中央バッファを共有
            auto *leftChild = new BoidUnit();
            auto *rightChild = new BoidUnit();
            leftChild->buf = node->buf;
            rightChild->buf = node->buf;
            node->children.push_back(leftChild);
            node->children.push_back(rightChild);

            // 再帰処理
            buildRecursive(leftChild, left, maxPerUnit, level + 1);
            buildRecursive(rightChild, right, maxPerUnit, level + 1);
        }
    }

    // 中間ノードには speciesId を設定しない
    node->speciesId = -1; // -1 を設定して無効化
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
      root->updateRecursive(dt);
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
    splitIndex = 0;
    mergeIndex = 0;
  }

  //一定フレームごとに木構造を再構築
  // if (frameCount % 17 == 0) { // 再構築頻度を調整
  //   build(maxBoidsPerUnit, 0);
  //   return;
  // }

  // 分割と結合の処理
  if (!leafCache.empty()) {
    // 分割
    for (int i = 0; i < 5 && splitIndex < (int)leafCache.size();
         ++i, ++splitIndex) {
      BoidUnit *u = leafCache[splitIndex];
      if (u && u->needsSplit(40.0f, 0.5f, maxBoidsPerUnit)) {
        u->splitInPlace(maxBoidsPerUnit);
        leafCache.clear();
        break;
      }
    }

    // 結合
    for (int i = 0; i < 5 && mergeIndex < (int)leafCache.size();
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