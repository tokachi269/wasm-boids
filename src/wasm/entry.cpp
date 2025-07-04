#include "entry.h"
#include "boids_tree.h"
#include "scale_utils.h"
#include <iostream>

void Entry::run() {
  std::cout << "WebAssembly entry point initialized!" << std::endl;
}

extern "C" {
void build(int maxPerUnit = 16, int level = 0) {
  BoidTree::instance().build(maxPerUnit, level);
}
uintptr_t posPtr() { return BoidTree::instance().getPositionsPtr(); }
uintptr_t velPtr() { return BoidTree::instance().getVelocitiesPtr(); }
uintptr_t oriPtr() { return BoidTree::instance().getOrientationsPtr(); }
int boidCount() { return BoidTree::instance().getBoidCount(); }
void update(float dt) { BoidTree::instance().update(dt); }
void setFlockSize(int newSize, float posRange, float velRange) {
  BoidTree::instance().setFlockSize(newSize, posRange, velRange);
}

void setSpeciesParams(const SpeciesParams &params,
                      float spatialScale /*=1.0f*/) {
  BoidTree::instance().setGlobalSpeciesParams(
      scaledParams(params, spatialScale));
}
uintptr_t boidUnitMappingPtr() {
  static std::vector<std::pair<int, int>> boidUnitMappingVec;
  boidUnitMappingVec.clear();
  const auto& mapping = BoidTree::instance().collectBoidUnitMapping();
  boidUnitMappingVec.reserve(mapping.size());
  for (const auto& kv : mapping) {
    boidUnitMappingVec.push_back(kv);
  }
  return reinterpret_cast<uintptr_t>(boidUnitMappingVec.data());
}
}
