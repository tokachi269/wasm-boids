#pragma once

#include <glm/glm.hpp>

namespace obstacle_field {

struct SurfaceQuery {
  bool active = false;
  float signedDistance = 0.0f;   // positive: outside, negative: penetrating
  glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
  float blendDistance = 0.0f;    // range where we start steering away
  float stiffness = 0.0f;        // base push strength (acceleration scale)
  float damping = 0.0f;          // velocity braking along the normal
};

SurfaceQuery sampleSurface(const glm::vec3 &position);

glm::vec3 computeAvoidance(const glm::vec3 &position, const glm::vec3 &velocity);

void resolvePenetration(glm::vec3 &position, glm::vec3 &velocity);

void configureGroundPlane(bool enabled, float height, float blendDistance,
                          float stiffness, float damping);

} // namespace obstacle_field
