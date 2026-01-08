#include "entry.h"
#include "boids_simulation.h"
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
    BoidSimulation::instance().update(dt);
  }

  gStepState.positions = BoidSimulation::instance().getPositionsPtr();
  gStepState.velocities = BoidSimulation::instance().getVelocitiesPtr();
  gStepState.orientations = BoidSimulation::instance().getOrientationsPtr();
  gStepState.count = BoidSimulation::instance().getBoidCount();

  return reinterpret_cast<uintptr_t>(&gStepState);
}

float currentFirstBoidX() { return debugFirstBoidX(); }

// 空間インデックス（現状は BoidUnit ツリー）を再構築。
// maxBoidsPerUnit は BoidSimulation 側の保持値を使う。
void build() { BoidSimulation::instance().build(); }
uintptr_t posPtr() { return BoidSimulation::instance().getPositionsPtr(); }
uintptr_t velPtr() { return BoidSimulation::instance().getVelocitiesPtr(); }
uintptr_t oriPtr() { return BoidSimulation::instance().getOrientationsPtr(); }
int boidCount() { return BoidSimulation::instance().getBoidCount(); }
void update(float dt) { BoidSimulation::instance().update(dt); }
void setFlockSize(int newSize, float posRange, float velRange) {
  BoidSimulation::instance().setFlockSize(newSize, posRange, velRange);
}

void setSpeciesParams(const SpeciesParams &params,
                      float spatialScale /*=1.0f*/) {
  BoidSimulation::instance().setGlobalSpeciesParams(
      scaledParams(params, spatialScale));
}
uintptr_t boidUnitMappingPtr() {
  static std::vector<std::pair<int, int>> boidUnitMappingVec;
  boidUnitMappingVec.clear();
  const auto &mapping = BoidSimulation::instance().collectBoidUnitMapping();
  boidUnitMappingVec.reserve(mapping.size());
  for (const auto &kv : mapping) {
    boidUnitMappingVec.push_back(kv);
  }
  return reinterpret_cast<uintptr_t>(boidUnitMappingVec.data());
}

uintptr_t speciesIdsPtr() {
  const auto &ids = BoidSimulation::instance().buf.speciesIds;
  if (ids.empty()) {
    return 0;
  }
  return reinterpret_cast<uintptr_t>(ids.data());
}

uintptr_t unitSimpleDensityPtr() {
  return BoidSimulation::instance().getUnitSimpleDensityPtr();
}

int unitSimpleDensityCount() {
  return BoidSimulation::instance().getUnitSimpleDensityCount();
}

uintptr_t EMSCRIPTEN_KEEPALIVE speciesEnvelopesPtr() {
  return BoidSimulation::instance().getSpeciesEnvelopePtr();
}

int EMSCRIPTEN_KEEPALIVE speciesEnvelopesCount() {
  return BoidSimulation::instance().getSpeciesEnvelopeCount();
}

// Species clusters debug export
// 1クラスターあたり 6 float: speciesId, center.xyz, radius, weight
uintptr_t EMSCRIPTEN_KEEPALIVE speciesClustersPtr() {
  return BoidSimulation::instance().getSpeciesClustersPtr();
}

int EMSCRIPTEN_KEEPALIVE speciesClustersCount() {
  return BoidSimulation::instance().getSpeciesClustersCount();
}

// Species school clusters debug export
// 1クラスターあたり 6 float: speciesId, center.xyz, radius, weight
uintptr_t EMSCRIPTEN_KEEPALIVE speciesSchoolClustersPtr() {
  return BoidSimulation::instance().getSpeciesSchoolClustersPtr();
}

int EMSCRIPTEN_KEEPALIVE speciesSchoolClustersCount() {
  return BoidSimulation::instance().getSpeciesSchoolClustersCount();
}

void syncReadToWriteBuffers() {
  BoidSimulation::instance().buf.syncWriteFromRead();
}
}
