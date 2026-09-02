#include "boids/boids_world.h"

#include <utility>

#include "boids_simulation.h"

namespace boids {

BoidsWorld::BoidsWorld(const BoidsWorldConfig &config)
    : config_(config), simulation_(std::make_unique<BoidSimulation>()) {
  configure(config_);
}

BoidsWorld::~BoidsWorld() = default;

BoidsWorld::BoidsWorld(BoidsWorld &&) noexcept = default;

BoidsWorld &BoidsWorld::operator=(BoidsWorld &&) noexcept = default;

void BoidsWorld::configure(const BoidsWorldConfig &config) {
  config_ = config;
  simulation_->setMaxBoidsPerUnit(config_.maxBoidsPerUnit);
  simulation_->setRandomSeed(config_.seed);
  simulation_->setFixedTimeStep(config_.fixedTimeStep);
}

void BoidsWorld::reset(const std::vector<SpeciesParams> &speciesParams) {
  simulation_->initializeBoids(speciesParams, config_.positionRange,
                               config_.velocityRange);
  simulation_->setMaxBoidsPerUnit(config_.maxBoidsPerUnit);
  simulation_->build();
}

void BoidsWorld::resize(int newSize) {
  simulation_->setFlockSize(newSize, config_.positionRange,
                            config_.velocityRange);
}

void BoidsWorld::step(float dt) { simulation_->update(dt); }

void BoidsWorld::stepFixed(float realDt) {
  simulation_->updateFixedStep(config_.fixedTimeStep, realDt);
}

void BoidsWorld::rebuildSpatialIndex() { simulation_->build(); }

void BoidsWorld::setSeed(uint32_t seed) {
  config_.seed = seed;
  simulation_->setRandomSeed(seed);
}

uint32_t BoidsWorld::seed() const { return simulation_->getRandomSeed(); }

void BoidsWorld::setFixedTimeStep(float dt) {
  config_.fixedTimeStep = dt > 0.0f ? dt : (1.0f / 60.0f);
  simulation_->setFixedTimeStep(config_.fixedTimeStep);
}

float BoidsWorld::fixedTimeStep() const {
  return simulation_->getFixedTimeStep();
}

int BoidsWorld::boidCount() const { return simulation_->getBoidCount(); }

std::span<const glm::vec3> BoidsWorld::positions() const {
  const auto &buffers = simulation_->getBuffers();
  return {buffers.positions.data(), buffers.positions.size()};
}

std::span<const glm::vec3> BoidsWorld::velocities() const {
  const auto &buffers = simulation_->getBuffers();
  return {buffers.velocities.data(), buffers.velocities.size()};
}

std::span<const glm::quat> BoidsWorld::orientations() const {
  const auto &buffers = simulation_->getBuffers();
  return {buffers.orientations.data(), buffers.orientations.size()};
}

std::span<const int> BoidsWorld::speciesIds() const {
  const auto &buffers = simulation_->getBuffers();
  return {buffers.speciesIds.data(), buffers.speciesIds.size()};
}

BoidsWorld::PhaseTimings BoidsWorld::phaseTimings() const {
  const auto timings = simulation_->getPhaseTimings();
  PhaseTimings result{};
  for (int i = 0; i < BoidSimulation::kPhaseCount; ++i) {
    result.ms[i] = timings.ms[i];
    result.calls[i] = timings.calls[i];
  }
  return result;
}

BoidsWorld::ParallelTimings BoidsWorld::parallelTimings() const {
  const auto timings = simulation_->getParallelTimings();
  ParallelTimings result{};
  for (int i = 0; i < BoidSimulation::kParallelPhaseCount; ++i) {
    result.taskMs[i] = timings.taskMs[i];
    result.maxTaskMs[i] = timings.maxTaskMs[i];
    result.minTaskMs[i] = timings.minTaskMs[i];
    result.worstMaxOverMean[i] = timings.worstMaxOverMean[i];
    result.tasks[i] = timings.tasks[i];
    result.frames[i] = timings.frames[i];
  }
  return result;
}

void BoidsWorld::resetPhaseTimings() { simulation_->resetPhaseTimings(); }

void BoidsWorld::setParallelTimingEnabled(bool enabled) {
  simulation_->setParallelTimingEnabled(enabled);
}

void BoidsWorld::beginLocalitySample() { simulation_->beginLocalitySample(); }

void BoidsWorld::endLocalitySample() { simulation_->endLocalitySample(); }

BoidsWorld::LocalityStats BoidsWorld::localityStats() const {
  const auto stats = simulation_->getLocalityStats();
  LocalityStats result{};
  for (int kind = 0; kind < 2; ++kind) {
    result.distanceSum[kind] = stats.distanceSum[kind];
    result.samples[kind] = stats.samples[kind];
    for (int bucket = 0; bucket < 6; ++bucket) {
      result.buckets[kind][bucket] = stats.buckets[kind][bucket];
    }
  }
  return result;
}

} // namespace boids
