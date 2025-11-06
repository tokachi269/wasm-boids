#pragma once

#include "boids_buffers.h"
#include "spatial_index.h"
#include <glm/glm.hpp>
#include <atomic>
#include <cstddef>
#include <utility>
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

  // center から最大 maxRadius 以内で距離が近い順に最大 maxCount 個の
  // Boid インデックスを outIndices/outDistancesSq に格納する。
  // 戻り値は格納した要素数。maxRadius <= 0 の場合は半径制限なし。
  int gatherNearest(const glm::vec3 &center, int maxCount, float maxRadius,
                    int *outIndices, float *outDistancesSq) const;

  // 与えられた Boid index が所属する葉ノードのメンバーリストを取得する。
  // true を返す場合、outIndices は葉内の連続領域を指し、outCount は要素数。
  bool getLeafMembers(int boidIndex, const int *&outIndices,
                      int &outCount) const;

  struct QueryStats {
    int queries = 0;
    int nodesVisited = 0;
    int leavesVisited = 0;
    int boidsConsidered = 0;
    int maxQueueSize = 0;
  };

  void resetQueryStats() const;
  QueryStats consumeQueryStats() const;

private:
  struct QueueEntry {
    int nodeIndex = -1;
    float distSq = 0.0f;
  };

  struct QueryScratch {
    std::vector<QueueEntry> stack;
    std::vector<std::pair<float, int>> best;
  };

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
  std::vector<int> boidToLeafIndex_;
  struct QueryStatsAtomic {
    std::atomic<int> queries{0};
    std::atomic<int> nodesVisited{0};
    std::atomic<int> leavesVisited{0};
    std::atomic<int> boidsConsidered{0};
    std::atomic<int> maxQueueSize{0};
  };

  mutable QueryStatsAtomic stats_;

  int buildRecursive(int start, int end);
  static glm::vec3 vecMin(const glm::vec3 &a, const glm::vec3 &b);
  static glm::vec3 vecMax(const glm::vec3 &a, const glm::vec3 &b);
  static QueryScratch &threadScratch();
};
