#pragma once

#include "boids_buffers.h"
#include "spatial_index.h"
#include <glm/glm.hpp>
#include <vector>


// LBVH (Linear Bounding Volume Hierarchy) に基づく SpatialIndex 実装。
// BoidTree とは独立に SoABuffers を直接参照して球クエリを提供する。
class LbvhIndex final : public SpatialIndex {
public:
  explicit LbvhIndex(int maxLeafSize = 32);

  // SoA バッファから LBVH 構造を再構築する。
  // positions が空の場合は内部状態をクリアする。
  void build(const SoABuffers &buffers);

  void forEachLeaf(const LeafVisitor &visitor) const override;
  void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                     const LeafVisitor &visitor) const override;

private:
  struct Node {
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};
    int left = -1;
    int right = -1;
    int begin = -1; // leaf: leafIndexStorage[begin, end)
    int end = -1;
    bool isLeaf = false;
  };

  struct LeafRecord {
    int nodeIndex = -1;
    int offset = 0;
    int count = 0;
  };

  const SoABuffers *buffers_ = nullptr;
  int maxLeafSize_ = 32;

  std::vector<int> mortonOrder_;        // Boid インデックス（モートン順）
  std::vector<int> leafIndexStorage_;   // 連続した Boid インデックス配列
  std::vector<Node> nodes_;             // BVH ノード配列（0 がルート）
  std::vector<LeafRecord> leafRecords_; // 迅速な葉列挙用キャッシュ

  int buildRecursive(int start, int end);
  static glm::vec3 vecMin(const glm::vec3 &a, const glm::vec3 &b);
  static glm::vec3 vecMax(const glm::vec3 &a, const glm::vec3 &b);
};
