#pragma once

#include "spatial_index.h"
#include "boids_buffers.h"
#include <vector>
#include <unordered_map>
#include <glm/vec3.hpp>

// ユニフォームグリッド（空間ハッシュ）による高速近傍探索
// 
// 設計方針：
// - セル幅 = 反応半径（cohesionRange）で最適化
// - 27セル探索（1+26近傍）による確実な近傍発見  
// - SoA + 段階的距離計算による高効率枝刈り
// - DBVH フォールバックとの連携を想定した軽量設計
//
// 利点：
// - 毎フレーム O(N) の安定性能
// - WASM で最も効率的な配列走査パターン
// - 実装・デバッグが容易
// - 密度分布に依存しない均一性能

class UniformGrid final : public SpatialIndex {
public:
  explicit UniformGrid(float cellSize);
  ~UniformGrid() override = default;

  // SpatialIndex インターフェース実装
  void forEachLeaf(const LeafVisitor &visitor) const override;
  void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                     const LeafVisitor &visitor) const override;

  // グリッド専用機能
  void build(const SoABuffers &buffers);
  void clear();
  
  // k-NN クエリ（DBVH フォールバック判定付き）
  struct QueryResult {
    int foundCount = 0;
    int candidatesConsidered = 0;
    bool usedFallback = false;
  };
  
  QueryResult gatherNearest(const glm::vec3 &center, int maxCount, 
                           float maxRadius, int *outIndices, 
                           float *outDistancesSq,
                           int fallbackThreshold = 256) const;

  // 統計情報
  struct GridStats {
    int totalCells = 0;
    int occupiedCells = 0;
    int maxCellOccupancy = 0;
    float avgCellOccupancy = 0.0f;
  };
  GridStats getStats() const;

private:
  struct CellKey {
    int x, y, z;
    
    bool operator==(const CellKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  
  struct CellKeyHash {
    std::size_t operator()(const CellKey &key) const {
      // FNV-1a ハッシュによる高品質な分散
      std::size_t hash = 2166136261u;
      hash ^= static_cast<std::size_t>(key.x);
      hash *= 16777619u;
      hash ^= static_cast<std::size_t>(key.y);  
      hash *= 16777619u;
      hash ^= static_cast<std::size_t>(key.z);
      hash *= 16777619u;
      return hash;
    }
  };

  CellKey positionToCell(const glm::vec3 &pos) const;
  void gatherCandidatesFromCells(const CellKey &centerCell, float maxRadiusSq,
                                const glm::vec3 &queryPos,
                                std::vector<int> &candidates) const;

  float cellSize_;
  float invCellSize_;
  const SoABuffers *buffers_ = nullptr;
  
  // セル → Boid インデックス配列のマッピング
  mutable std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells_;
};