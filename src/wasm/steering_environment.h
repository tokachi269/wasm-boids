#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "boids/steering.h"

struct ObstacleAvoidanceMemory {
  int32_t obstacleId = -1;
  glm::vec3 tangent = glm::vec3(0.0f);
};

class SteeringEnvironment {
public:
  struct SurfaceSample {
    bool active = false;
    float signedDistance = 0.0f;
    glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
  };

  SteeringEnvironment();

  void setGuides(const std::vector<boids::Guide> &guides);
  void setObstacles(const std::vector<boids::Obstacle> &obstacles);
  void configureGroundPlane(bool enabled, float height,
                            float influenceDistance, float strength,
                            float damping);

  const std::vector<boids::Guide> &guides() const { return guides_; }
  const std::vector<boids::Obstacle> &obstacles() const { return obstacles_; }
  bool usesAvoidanceMemory() const { return usesAvoidanceMemory_; }

  glm::vec3 computeSteering(const glm::vec3 &position,
                            const glm::vec3 &velocity, uint32_t stableId,
                            ObstacleAvoidanceMemory &memory) const;
  void resolvePenetration(glm::vec3 &position, glm::vec3 &velocity) const;

  SurfaceSample sampleSurface(const boids::Obstacle &obstacle,
                              const glm::vec3 &position) const;

  static constexpr int kGuideRecordFloats = 7;
  static constexpr int kObstacleRecordFloats = 17;
  float *resizeGuideInput(std::size_t count);
  float *resizeObstacleInput(std::size_t count);
  bool commitGuideInput(std::size_t count);
  bool commitObstacleInput(std::size_t count);

private:
  glm::vec3 computeGuideSteering(const glm::vec3 &position) const;
  glm::vec3 computeObstacleSteering(const glm::vec3 &position,
                                    const glm::vec3 &velocity,
                                    uint32_t stableId,
                                    ObstacleAvoidanceMemory &memory) const;

  bool groundEnabled_ = true;
  boids::Obstacle groundPlane_;
  std::vector<boids::Guide> guides_;
  std::vector<boids::Obstacle> obstacles_;
  std::vector<float> guideInput_;
  std::vector<float> obstacleInput_;
  bool usesAvoidanceMemory_ = false;
};
