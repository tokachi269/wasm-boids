#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "species_params.h"

class BoidSimulation;

namespace boids {

struct BoidsWorldConfig {
  float positionRange = 3.0f;
  float velocityRange = 0.25f;
  int maxBoidsPerUnit = 16;
  uint32_t seed = 5489u;
  float fixedTimeStep = 1.0f / 60.0f;
};

class BoidsWorld {
public:
  explicit BoidsWorld(const BoidsWorldConfig &config = {});
  ~BoidsWorld();

  BoidsWorld(const BoidsWorld &) = delete;
  BoidsWorld &operator=(const BoidsWorld &) = delete;
  BoidsWorld(BoidsWorld &&) noexcept;
  BoidsWorld &operator=(BoidsWorld &&) noexcept;

  void configure(const BoidsWorldConfig &config);
  const BoidsWorldConfig &config() const { return config_; }

  void reset(const std::vector<SpeciesParams> &speciesParams);
  void resize(int newSize);
  void step(float dt);
  void stepFixed(float realDt);
  void rebuildSpatialIndex();

  void setSeed(uint32_t seed);
  uint32_t seed() const;

  void setFixedTimeStep(float dt);
  float fixedTimeStep() const;

  int boidCount() const;
  std::span<const glm::vec3> positions() const;
  std::span<const glm::vec3> velocities() const;
  std::span<const glm::quat> orientations() const;
  std::span<const int> speciesIds() const;

private:
  BoidsWorldConfig config_;
  std::unique_ptr<BoidSimulation> simulation_;
};

} // namespace boids