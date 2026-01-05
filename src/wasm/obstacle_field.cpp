#include "obstacle_field.h"

#include <algorithm>

namespace obstacle_field {
namespace {
struct GroundPlaneConfig {
  bool enabled = true;
  float height = 0.0f;
  float blendDistance = 2.0f;
  float stiffness = 18.0f;
  float damping = 8.0f;
};

GroundPlaneConfig gGround;
constexpr float kPenetrationEpsilon = 1e-4f;

} // namespace

SurfaceQuery sampleSurface(const glm::vec3 &position) {
  SurfaceQuery query;
  if (!gGround.enabled) {
    return query;
  }

  const float signedDistance = position.y - gGround.height;
  query.active = true;
  query.signedDistance = signedDistance;
  query.normal = glm::vec3(0.0f, 1.0f, 0.0f);
  query.blendDistance = gGround.blendDistance;
  query.stiffness = gGround.stiffness;
  query.damping = gGround.damping;
  return query;
}

glm::vec3 computeAvoidance(const glm::vec3 &position,
                            const glm::vec3 &velocity) {
  const SurfaceQuery query = sampleSurface(position);
  if (!query.active) {
    return glm::vec3(0.0f);
  }
  if (query.signedDistance >= query.blendDistance) {
    return glm::vec3(0.0f);
  }

  const float depth = query.blendDistance - query.signedDistance;
  const float proximity = std::clamp(depth / std::max(query.blendDistance, 1e-4f), 0.0f, 1.0f);
  const float eased = proximity * proximity * (3.0f - 2.0f * proximity);
  float push = eased * query.stiffness;

  // 減速: 法線方向へ向かう速度を抑えることで滑らかに回避する。
  const float approachSpeed = -glm::dot(velocity, query.normal);
  if (approachSpeed > 0.0f) {
    push += approachSpeed * query.damping;
  }

  return query.normal * push;
}

void resolvePenetration(glm::vec3 &position, glm::vec3 &velocity) {
  const SurfaceQuery query = sampleSurface(position);
  if (!query.active) {
    return;
  }
  if (query.signedDistance >= 0.0f) {
    return;
  }

  const float correction = -(query.signedDistance) + kPenetrationEpsilon;
  position += query.normal * correction;

  const float normalSpeed = glm::dot(velocity, query.normal);
  if (normalSpeed < 0.0f) {
    velocity -= query.normal * normalSpeed;
  }
}

void configureGroundPlane(bool enabled, float height, float blendDistance,
                          float stiffness, float damping) {
  gGround.enabled = enabled;
  gGround.height = height;
  gGround.blendDistance = std::max(blendDistance, 0.0f);
  gGround.stiffness = std::max(stiffness, 0.0f);
  gGround.damping = std::max(damping, 0.0f);
}

} // namespace obstacle_field
