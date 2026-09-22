#pragma once

#include <cstdint>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace boids {

struct Guide {
  bool enabled = true;
  glm::vec3 target = glm::vec3(0.0f);
  float freeRadius = 0.0f;
  float responseDistance = 1.0f;
  float strength = 0.0f;
};

enum class ObstacleShape : int {
  Plane = 0,
  Sphere = 1,
  Capsule = 2,
  Box = 3,
};

struct Obstacle {
  bool enabled = true;
  // Negative values are replaced with the array index. Keep explicit IDs
  // stable while updating transforms so the chosen avoidance side persists.
  int32_t id = -1;
  ObstacleShape shape = ObstacleShape::Sphere;
  glm::vec3 position = glm::vec3(0.0f);
  glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

  // Sphere: x = radius
  // Capsule: x = radius, y = half length along local Y
  // Box: xyz = half extents
  // Plane: unused; local +Y is the allowed-side normal
  glm::vec3 size = glm::vec3(1.0f);

  float influenceDistance = 2.0f;
  float strength = 18.0f;
  float damping = 8.0f;
  float tangentStrength = 4.0f;
};

} // namespace boids
