#include "entry.h"
#include "boids_tree.h"
#include "scale_utils.h"
#include <cstdint>
#include <iostream>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#ifndef __EMSCRIPTEN__
#include "native_simulation.h"
#endif

void Entry::run() {
#ifdef __EMSCRIPTEN__
  std::cout << "WebAssembly entry point initialized!" << std::endl;
#else
  std::cout << "Starting native boids simulation..." << std::endl;
  NativeSimulation simulation;
  simulation.run();
#endif
}

extern "C" {
struct SimulationStepState {
  uintptr_t positions;
  uintptr_t velocities;
  uintptr_t orientations;
  int count;
};

static SimulationStepState gStepState{};

float debugFirstBoidX() {
  if (gStepState.count <= 0) {
    return 0.0f;
  }
  auto ptr = reinterpret_cast<const float *>(gStepState.positions);
  return ptr ? ptr[0] : 0.0f;
}

uintptr_t stepSimulation(float dt) {
  if (dt > 0.0f) {
    BoidTree::instance().update(dt);
  }

  gStepState.positions = BoidTree::instance().getPositionsPtr();
  gStepState.velocities = BoidTree::instance().getVelocitiesPtr();
  gStepState.orientations = BoidTree::instance().getOrientationsPtr();
  gStepState.count = BoidTree::instance().getBoidCount();

  return reinterpret_cast<uintptr_t>(&gStepState);
}

float currentFirstBoidX() { return debugFirstBoidX(); }

void build(int maxPerUnit = 16) { BoidTree::instance().build(maxPerUnit); }
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
  const auto &mapping = BoidTree::instance().collectBoidUnitMapping();
  boidUnitMappingVec.reserve(mapping.size());
  for (const auto &kv : mapping) {
    boidUnitMappingVec.push_back(kv);
  }
  return reinterpret_cast<uintptr_t>(boidUnitMappingVec.data());
}

uintptr_t speciesIdsPtr() {
  const auto &ids = BoidTree::instance().buf.speciesIds;
  if (ids.empty()) {
    return 0;
  }
  return reinterpret_cast<uintptr_t>(ids.data());
}

uintptr_t unitSimpleDensityPtr() {
  return BoidTree::instance().getUnitSimpleDensityPtr();
}

int unitSimpleDensityCount() {
  return BoidTree::instance().getUnitSimpleDensityCount();
}

uintptr_t EMSCRIPTEN_KEEPALIVE speciesEnvelopesPtr() {
  return BoidTree::instance().getSpeciesEnvelopePtr();
}

int EMSCRIPTEN_KEEPALIVE speciesEnvelopesCount() {
  return BoidTree::instance().getSpeciesEnvelopeCount();
}

// Species clusters debug export
// 1クラスターあたり 6 float: speciesId, center.xyz, radius, weight
uintptr_t EMSCRIPTEN_KEEPALIVE speciesClustersPtr() {
  return BoidTree::instance().getSpeciesClustersPtr();
}

int EMSCRIPTEN_KEEPALIVE speciesClustersCount() {
  return BoidTree::instance().getSpeciesClustersCount();
}

// Species school clusters debug export
// 1クラスターあたり 6 float: speciesId, center.xyz, radius, weight
uintptr_t EMSCRIPTEN_KEEPALIVE speciesSchoolClustersPtr() {
  return BoidTree::instance().getSpeciesSchoolClustersPtr();
}

int EMSCRIPTEN_KEEPALIVE speciesSchoolClustersCount() {
  return BoidTree::instance().getSpeciesSchoolClustersCount();
}

void syncReadToWriteBuffers() {
  BoidTree::instance().buf.syncWriteFromRead();
}
}
