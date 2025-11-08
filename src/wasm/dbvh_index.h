#pragma once

#include "boids_buffers.h"

#include <glm/vec3.hpp>

#include <cstddef>
#include <vector>

// 動的BVH (Dynamic Bounding Volume Hierarchy) による予備インデックス。
// ユニフォームグリッドで候補が溢れた際のバックアップとして枝刈り検索を提供する。
class DbvhIndex {
public:
  DbvhIndex() = default;

  void clear();

  // SoA バッファと半径を与えて木を再構築、あるいは差分更新する。
  // velocityPadding は速度に対する余白（fat AABB）を設定するパラメータ。
  void sync(const SoABuffers &buffers, float baseRadius,
            float velocityPadding);

  // 球クエリ。半径内にある Boid インデックスを最大 maxOut 件列挙する。
  int gatherWithinRadius(const glm::vec3 &center, float radius,
                         int *outIndices, int maxOut) const;

  bool empty() const { return root_ == -1; }

private:
  struct Node {
    glm::vec3 aabbMin{0.0f};
    glm::vec3 aabbMax{0.0f};
    int parent = -1;
    int left = -1;
    int right = -1;
    int height = -1;
    int next = -1; // フリーリスト用リンク
    int boidIndex = -1;

    bool isLeaf() const { return left == -1; }
  };

  const SoABuffers *buffers_ = nullptr;
  std::vector<Node> nodes_;
  int root_ = -1;
  int freeList_ = -1;
  std::vector<int> handles_;

  void rebuild(const SoABuffers &buffers, float fatRadius);
  void ensureCapacity(std::size_t count);
  int allocateNode();
  void freeNode(int nodeIndex);
  void insertLeaf(int leafIndex);
  void removeLeaf(int leafIndex);
  void updateLeaf(int leafIndex, const glm::vec3 &pos, float fatRadius);
  int pickBestSibling(int leafIndex) const;
  int balance(int index);
  void fixUpwardsTree(int index);

  static glm::vec3 combineMin(const glm::vec3 &a, const glm::vec3 &b);
  static glm::vec3 combineMax(const glm::vec3 &a, const glm::vec3 &b);
  static float surfaceArea(const glm::vec3 &min, const glm::vec3 &max);
  static bool contains(const Node &node, const glm::vec3 &min,
                       const glm::vec3 &max);
  static float distanceSqAabb(const glm::vec3 &point, const glm::vec3 &min,
                              const glm::vec3 &max);
};
