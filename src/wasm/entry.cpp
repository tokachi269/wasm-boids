#include "entry.h"
#include "boids_simulation.h"
#include "boids_parallel_config.h"
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

int Entry::run(int argc, char **argv) {
#ifdef __EMSCRIPTEN__
  std::cout << "WebAssembly entry point initialized!" << std::endl;
  return 0;
#else
  NativeSimulation simulation;
  if (!simulation.configureFromCommandLine(argc, argv)) {
    return 2;
  }
  simulation.run();
  return 0;
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
  return BoidSimulation::instance().getSpeciesIdsPtr();
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
  BoidSimulation::instance().syncWriteFromReadBuffers();
}

void resetPhaseTimings() { BoidSimulation::instance().resetPhaseTimings(); }

double phaseTimingMs(int phase) {
  if (phase < 0 || phase >= BoidSimulation::kPhaseCount) {
    return 0.0;
  }
  return BoidSimulation::instance().getPhaseTimings().ms[phase];
}

int phaseTimingCalls(int phase) {
  if (phase < 0 || phase >= BoidSimulation::kPhaseCount) {
    return 0;
  }
  return static_cast<int>(
      BoidSimulation::instance().getPhaseTimings().calls[phase]);
}

double parallelTimingValue(int phase, int metric) {
  if (phase < 0 || phase >= BoidSimulation::kParallelPhaseCount) {
    return 0.0;
  }
  const auto timings = BoidSimulation::instance().getParallelTimings();
  switch (metric) {
  case 0:
    return timings.taskMs[phase];
  case 1:
    return timings.maxTaskMs[phase];
  case 2:
    return timings.minTaskMs[phase];
  case 3:
    return timings.worstMaxOverMean[phase];
  case 4:
    return static_cast<double>(timings.tasks[phase]);
  case 5:
    return static_cast<double>(timings.frames[phase]);
  default:
    return 0.0;
  }
}

void configureBenchmarkDiagnostics(unsigned int seed, int taskLimit,
                                   bool parallelTiming) {
  BoidSimulation::instance().setRandomSeed(seed);
  setBoidsMaxTasksOverride(taskLimit > 0 ? static_cast<std::size_t>(taskLimit)
                                         : 0);
  BoidSimulation::instance().setParallelTimingEnabled(parallelTiming);
}

void beginLocalitySample() { BoidSimulation::instance().beginLocalitySample(); }
void endLocalitySample() { BoidSimulation::instance().endLocalitySample(); }

double localityValue(int kind, int metric) {
  if (kind < 0 || kind > 1) {
    return 0.0;
  }
  const auto stats = BoidSimulation::instance().getLocalityStats();
  if (metric >= 0 && metric < 6) {
    return static_cast<double>(stats.buckets[kind][metric]);
  }
  if (metric == 6) {
    return static_cast<double>(stats.distanceSum[kind]);
  }
  if (metric == 7) {
    return static_cast<double>(stats.samples[kind]);
  }
  return 0.0;
}
}
