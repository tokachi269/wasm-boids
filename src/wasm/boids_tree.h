#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/vec3.hpp>

#include "boid.h"
#include "boid_unit.h"
#include "boids_buffers.h"
#include "lbvh_index.h"
#include "species_params.h"
#include "spatial_index.h"

// Boid群の管理とシミュレーション更新を行うクラス
class BoidTree : public SpatialIndex {
public:
  static BoidTree &instance();

  BoidTree();
  ~BoidTree();

  void setFlockSize(int newSize, float posRange, float velRange);
  void initializeBoids(const std::vector<SpeciesParams> &speciesParamsList,
                       float posRange, float velRange);
  void initializeBoidMemories(const std::vector<SpeciesParams> &speciesParamsList);
  void build(int maxPerUnit = 16);
  SpatialIndex &spatialIndex();
  const SpatialIndex &spatialIndex() const;
  void forEachLeaf(const LeafVisitor &visitor) const override;
  void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                     const LeafVisitor &visitor) const override;
  void update(float dt = 1.0f);

  LbvhIndex::QueryStats getLastQueryStats() const { return lastQueryStats_; }

  uintptr_t getPositionsPtr();
  uintptr_t getVelocitiesPtr();
  uintptr_t getOrientationsPtr();
  int getBoidCount() const;
  std::unordered_map<int, int> collectBoidUnitMapping();

  SpeciesParams getGlobalSpeciesParams(std::string species);
  void setGlobalSpeciesParams(const SpeciesParams &params);

  SoABuffers buf;

private:
  void setRenderPointersToReadBuffers();
  void setRenderPointersToWriteBuffers();
  void rebuildSpatialIndex();
  void resetSpatialIndexQuality();
  bool shouldRebuildForQuality(const LbvhIndex::QueryStats &stats);

  uintptr_t renderPositionsPtr_ = 0;
  uintptr_t renderVelocitiesPtr_ = 0;
  uintptr_t renderOrientationsPtr_ = 0;

  int frameCount = 0;
  int maxBoidsPerUnit = 16;
  LbvhIndex lbvhIndex_;
  LbvhIndex::QueryStats lastQueryStats_{};
  bool lbvhDirty_ = true;
  int framesSinceRebuild_ = 0;
  float cumulativeDisplacementSinceRebuild_ = 0.0f;
  float maxStepDisplacementSqSinceRebuild_ = 0.0f;
  float lastAverageDisplacement_ = 0.0f;
  float queryNodesEwma_ = 0.0f;
  float queryBoidsEwma_ = 0.0f;
  float queryNodesBaseline_ = 0.0f;
  float queryBoidsBaseline_ = 0.0f;
  int querySamplesSinceRebuild_ = 0;
  bool queryBaselineValid_ = false;
};

extern std::vector<SpeciesParams> globalSpeciesParams;
