#pragma once

#include <functional>
#include <glm/glm.hpp>
#include <vector>

// 近傍探索を高速化するための fat-AABB ベース動的 BVH。
class DynamicBVH {
public:
  struct Node {
    glm::vec3 aabbMin;
    glm::vec3 aabbMax;
    int parent = -1;
    int child1 = -1;
    int child2 = -1;
    int next = -1;
    int boidIndex = -1;

    bool isLeaf() const { return child1 == -1; }
  };

  DynamicBVH();

  void clear();
  int createProxy(const glm::vec3 &min, const glm::vec3 &max, int boidIndex);
  void destroyProxy(int proxyId);
  bool updateProxy(int proxyId, const glm::vec3 &min, const glm::vec3 &max,
                   const glm::vec3 &fatMargin);

  template <typename Callback>
  void query(const glm::vec3 &min, const glm::vec3 &max, Callback &&cb) const {
    if (root_ == -1)
      return;

    auto &stack = queryStack_;
    stack.clear();
    stack.push_back(root_);

    while (!stack.empty()) {
      int nodeId = stack.back();
      stack.pop_back();
      const Node &node = nodes_[nodeId];
      if (!overlaps(node.aabbMin, node.aabbMax, min, max))
        continue;

      if (node.isLeaf()) {
        // コールバックが false を返したら探索を打ち切る。
        if (!cb(node.boidIndex))
          return;
      } else {
        if (node.child1 != -1)
          stack.push_back(node.child1);
        if (node.child2 != -1)
          stack.push_back(node.child2);
      }
    }
  }

  const std::vector<Node> &nodes() const { return nodes_; }

private:
  int allocateNode();
  void freeNode(int nodeId);
  void insertLeaf(int leafId);
  void removeLeaf(int leafId);
  void fixUpwardsTree(int nodeId);
  int pickBestSibling(const glm::vec3 &leafMin, const glm::vec3 &leafMax) const;
  static bool overlaps(const glm::vec3 &aMin, const glm::vec3 &aMax,
                       const glm::vec3 &bMin, const glm::vec3 &bMax);
  static glm::vec3 combineMin(const glm::vec3 &aMin, const glm::vec3 &bMin);
  static glm::vec3 combineMax(const glm::vec3 &aMax, const glm::vec3 &bMax);
  static float surfaceArea(const glm::vec3 &min, const glm::vec3 &max);

  std::vector<Node> nodes_;
  mutable std::vector<int> queryStack_;
  int root_ = -1;
  int freeList_ = -1;
  int nodeCount_ = 0;
};
