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

  uintptr_t renderPositionsPtr_ = 0;
  uintptr_t renderVelocitiesPtr_ = 0;
  uintptr_t renderOrientationsPtr_ = 0;

  int frameCount = 0;
  int maxBoidsPerUnit = 16;
  LbvhIndex lbvhIndex_;
  LbvhIndex::QueryStats lastQueryStats_{};
};

extern std::vector<SpeciesParams> globalSpeciesParams;
