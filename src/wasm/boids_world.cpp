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

} // namespace boids