#pragma once

#include "boids_buffers.h"
#include "spatial_index.h"

#include <cstdint>
#include <glm/vec3.hpp>

#include <unordered_map>
#include <vector>

// ユニフォームグリッド（空間ハッシュ）による近傍探索インデックス。
// セル幅を一定に保ち、27 近傍セルを列挙することで Boid の局所探索を高速化する。
class UniformGridIndex : public SpatialIndex {
public:
  explicit UniformGridIndex(float cellSize = 50.0f);

  // セル幅（反応半径の目安）を変更する。値が極端に小さくならないよう clamp する。
  void setCellSize(float cellSize);

  // サンプリング時の疑似乱数シードを設定する。フレームごとに更新して均等化する。
  void setSamplingSeed(std::uint32_t seed);

  // 与えられた SoA バッファからグリッドを再構築する。
  void build(const SoABuffers &buffers);

  // 近傍探索：center の周囲から最大 maxCount 件を距離昇順で取得する。
  // maxRadius が 0 以下の場合はセル幅を基準とした動的半径を採用する。
  int gatherNearest(const glm::vec3 &center, int maxCount, float maxRadius,
                    int *outIndices, float *outDistancesSq) const;

  // 指定 Boid が所属するセルに含まれるインデックス配列を取得する。
  // セルが存在しない場合や無効インデックスでは false を返す。
  bool getLeafMembers(int boidIndex, const int *&outIndices,
                      int &count) const;

  void clear();

  void forEachLeaf(const LeafVisitor &visitor) const override;
  void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                     const LeafVisitor &visitor) const override;

private:
  struct CellKey {
    int x = 0;
    int y = 0;
    int z = 0;
  };

  struct Cell {
    std::vector<int> indices; // セルに所属する Boid インデックス
  };

  struct KeyHash {
    std::size_t operator()(const CellKey &key) const noexcept;
  };

  struct KeyEq {
    bool operator()(const CellKey &lhs, const CellKey &rhs) const noexcept;
  };

  CellKey cellFor(const glm::vec3 &position) const;

  float cellSize_ = 10.0f;
  float invCellSize_ = 1.0f / 10.0f;
  const SoABuffers *buffers_ = nullptr;
  std::uint32_t samplingSeed_ = 0;
  std::unordered_map<CellKey, Cell, KeyHash, KeyEq> cells_;
};
