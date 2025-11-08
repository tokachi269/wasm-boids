#pragma once

#include <cstddef>
#include <glm/vec3.hpp>

struct SoABuffers;
class UniformGridIndex;
class DbvhIndex;

// KD 木時代との互換性確保のために残しているダミー構造体。
// 空間インデックス刷新後もメタデータ保持に利用しない。
struct BoidUnit {};

// グリッドと DBVH の併用状況を記録する統計。
struct HybridNeighborStats {
  int queryCount = 0;
  int overflowCount = 0;
  int fallbackQueries = 0;
  int fallbackAdded = 0;
};

// グリッドを優先しつつ高密度時のみ DBVH にフォールバックする近傍プロバイダ。
struct GridDbvhNeighborProvider {
  GridDbvhNeighborProvider(const SoABuffers &buffers,
                           const UniformGridIndex &gridIndex,
                           const DbvhIndex *dbvhIndex, int fallbackThreshold,
                           int fallbackLimit, float fallbackRadiusScale,
                           HybridNeighborStats *statsRef);

  int gatherNearest(const glm::vec3 &center, int maxCount, float maxRadius,
                    int *outIndices, float *outDistancesSq) const;
  bool getLeafMembers(int boidIndex, const int *&outIndices, int &count) const;

  const SoABuffers &buffers;
  const UniformGridIndex &grid;
  const DbvhIndex *dbvh;
  int threshold;
  int fallbackCapacity;
  float radiusScale;
  HybridNeighborStats *stats;
};

// ユニフォームグリッドインデックスを用いた近傍相互作用計算。
void computeBoidInteractionsRange(SoABuffers &buf,
                                  const GridDbvhNeighborProvider &neighbors,
                                  int begin, int end, float dt);

// 速度・位置・姿勢などの運動学的更新を SoA バッファ上で処理。
void updateBoidKinematicsRange(SoABuffers &buf, int begin, int end, float dt);