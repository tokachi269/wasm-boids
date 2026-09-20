#define GLM_ENABLE_EXPERIMENTAL
#include "steering_environment.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

namespace {

constexpr float kEpsilon = 1e-6f;
constexpr float kPenetrationEpsilon = 1e-4f;
constexpr int32_t kGroundObstacleId = std::numeric_limits<int32_t>::min();

float smoothResponse(float value) {
  const float t = std::clamp(value, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

glm::quat safeRotation(const glm::quat &rotation) {
  const float length2 = glm::dot(rotation, rotation);
  if (length2 <= kEpsilon) {
    return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  }
  return rotation * (1.0f / std::sqrt(length2));
}

glm::vec3 safeNormal(const glm::vec3 &value, const glm::vec3 &fallback) {
  const float length2 = glm::length2(value);
  if (length2 <= kEpsilon) {
    return fallback;
  }
  return value * (1.0f / std::sqrt(length2));
}

uint32_t hash32(uint32_t value) {
  value ^= value >> 16;
  value *= 0x7feb352dU;
  value ^= value >> 15;
  value *= 0x846ca68bU;
  value ^= value >> 16;
  return value;
}

glm::vec3 deterministicTangent(const glm::vec3 &normal, uint32_t stableId,
                               int32_t obstacleId) {
  const glm::vec3 reference = std::abs(normal.y) < 0.85f
                                  ? glm::vec3(0.0f, 1.0f, 0.0f)
                                  : glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 tangent = safeNormal(glm::cross(normal, reference),
                                 glm::vec3(1.0f, 0.0f, 0.0f));
  const uint32_t key = hash32(stableId ^ (static_cast<uint32_t>(obstacleId) *
                                         0x9e3779b9U));
  if ((key & 1U) != 0U) {
    tangent = -tangent;
  }
  return tangent;
}

boids::Obstacle sanitizedObstacle(const boids::Obstacle &source) {
  boids::Obstacle result = source;
  result.rotation = safeRotation(result.rotation);
  result.size = glm::max(glm::abs(result.size), glm::vec3(kEpsilon));
  result.influenceDistance = std::max(result.influenceDistance, 0.0f);
  result.strength = std::max(result.strength, 0.0f);
  result.damping = std::max(result.damping, 0.0f);
  result.tangentStrength = std::max(result.tangentStrength, 0.0f);
  return result;
}

} // namespace

SteeringEnvironment::SteeringEnvironment() {
  groundPlane_.enabled = true;
  groundPlane_.id = kGroundObstacleId;
  groundPlane_.shape = boids::ObstacleShape::Plane;
  groundPlane_.position = glm::vec3(0.0f);
  groundPlane_.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  groundPlane_.influenceDistance = 2.0f;
  groundPlane_.strength = 18.0f;
  groundPlane_.damping = 8.0f;
  groundPlane_.tangentStrength = 0.0f;
}

void SteeringEnvironment::setGuides(
    const std::vector<boids::Guide> &guides) {
  guides_ = guides;
  for (boids::Guide &guide : guides_) {
    guide.freeRadius = std::max(guide.freeRadius, 0.0f);
    guide.responseDistance = std::max(guide.responseDistance, kEpsilon);
    guide.strength = std::max(guide.strength, 0.0f);
  }
}

void SteeringEnvironment::setObstacles(
    const std::vector<boids::Obstacle> &obstacles) {
  obstacles_.clear();
  obstacles_.reserve(obstacles.size());
  usesAvoidanceMemory_ = false;
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const boids::Obstacle &obstacle = obstacles[index];
    obstacles_.push_back(sanitizedObstacle(obstacle));
    boids::Obstacle &stored = obstacles_.back();
    if (stored.id < 0) {
      stored.id = static_cast<int32_t>(index);
    }
    usesAvoidanceMemory_ = usesAvoidanceMemory_ ||
                           (stored.enabled && stored.tangentStrength > 0.0f);
  }
}

void SteeringEnvironment::configureGroundPlane(bool enabled, float height,
                                                float influenceDistance,
                                                float strength,
                                                float damping) {
  groundEnabled_ = enabled;
  groundPlane_.enabled = enabled;
  groundPlane_.position = glm::vec3(0.0f, height, 0.0f);
  groundPlane_.influenceDistance = std::max(influenceDistance, 0.0f);
  groundPlane_.strength = std::max(strength, 0.0f);
  groundPlane_.damping = std::max(damping, 0.0f);
}

SteeringEnvironment::SurfaceSample SteeringEnvironment::sampleSurface(
    const boids::Obstacle &obstacle, const glm::vec3 &position) const {
  SurfaceSample sample;
  if (!obstacle.enabled) {
    return sample;
  }

  sample.active = true;
  const glm::quat rotation = obstacle.rotation;
  const glm::quat inverseRotation = glm::conjugate(rotation);
  const glm::vec3 local = inverseRotation * (position - obstacle.position);

  switch (obstacle.shape) {
  case boids::ObstacleShape::Plane: {
    sample.signedDistance = local.y;
    sample.normal = rotation * glm::vec3(0.0f, 1.0f, 0.0f);
    break;
  }
  case boids::ObstacleShape::Sphere: {
    const float radius = obstacle.size.x;
    const float distance2 = glm::length2(local);
    if (distance2 > kEpsilon) {
      const float distance = std::sqrt(distance2);
      sample.signedDistance = distance - radius;
      sample.normal = rotation * (local / distance);
    } else {
      sample.signedDistance = -radius;
      sample.normal = rotation * glm::vec3(0.0f, 1.0f, 0.0f);
    }
    break;
  }
  case boids::ObstacleShape::Capsule: {
    const float radius = obstacle.size.x;
    const float halfLength = obstacle.size.y;
    const glm::vec3 closest(0.0f,
                            std::clamp(local.y, -halfLength, halfLength),
                            0.0f);
    const glm::vec3 offset = local - closest;
    const float distance2 = glm::length2(offset);
    if (distance2 > kEpsilon) {
      const float distance = std::sqrt(distance2);
      sample.signedDistance = distance - radius;
      sample.normal = rotation * (offset / distance);
    } else {
      sample.signedDistance = -radius;
      sample.normal = rotation * glm::vec3(1.0f, 0.0f, 0.0f);
    }
    break;
  }
  case boids::ObstacleShape::Box: {
    const glm::vec3 halfExtents = obstacle.size;
    const glm::vec3 outside = glm::max(glm::abs(local) - halfExtents,
                                      glm::vec3(0.0f));
    const float outsideDistance2 = glm::length2(outside);
    const glm::vec3 closest = glm::clamp(local, -halfExtents, halfExtents);
    if (outsideDistance2 > kEpsilon) {
      const glm::vec3 offset = local - closest;
      const float outsideDistance = std::sqrt(outsideDistance2);
      sample.signedDistance = outsideDistance;
      sample.normal = rotation * (offset / outsideDistance);
    } else {
      const glm::vec3 margin = halfExtents - glm::abs(local);
      int axis = 0;
      if (margin.y < margin.x) {
        axis = 1;
      }
      if (margin.z < margin[axis]) {
        axis = 2;
      }
      glm::vec3 localNormal(0.0f);
      localNormal[axis] = local[axis] < 0.0f ? -1.0f : 1.0f;
      sample.signedDistance = -margin[axis];
      sample.normal = rotation * localNormal;
    }
    break;
  }
  }

  sample.normal = safeNormal(sample.normal, glm::vec3(0.0f, 1.0f, 0.0f));
  return sample;
}

glm::vec3 SteeringEnvironment::computeGuideSteering(
    const glm::vec3 &position) const {
  glm::vec3 steering(0.0f);
  for (const boids::Guide &guide : guides_) {
    if (!guide.enabled || guide.strength <= 0.0f) {
      continue;
    }
    const glm::vec3 toTarget = guide.target - position;
    const float distance2 = glm::length2(toTarget);
    if (distance2 <= guide.freeRadius * guide.freeRadius ||
        distance2 <= kEpsilon) {
      continue;
    }
    const float distance = std::sqrt(distance2);
    const float excess = distance - guide.freeRadius;
    const float response = smoothResponse(excess / guide.responseDistance);
    steering += (toTarget / distance) * (guide.strength * response);
  }
  return steering;
}

glm::vec3 SteeringEnvironment::computeObstacleSteering(
    const glm::vec3 &position, const glm::vec3 &velocity, uint32_t stableId,
    ObstacleAvoidanceMemory &memory) const {
  glm::vec3 steering(0.0f);
  const int32_t previousObstacleId = memory.obstacleId;
  const glm::vec3 previousTangent = memory.tangent;
  bool previousObstacleActive = false;
  bool selectedObstacle = false;
  float memoryPriority = std::numeric_limits<float>::max();

  const auto accumulate = [&](const boids::Obstacle &obstacle,
                              const SurfaceSample &sample,
                              glm::vec3 &result) {
    if (!obstacle.enabled || obstacle.influenceDistance <= 0.0f) {
      return;
    }
    if (!sample.active || sample.signedDistance >= obstacle.influenceDistance) {
      return;
    }

    const float proximity = std::clamp(
        (obstacle.influenceDistance - sample.signedDistance) /
            std::max(obstacle.influenceDistance, kEpsilon),
        0.0f, 1.0f);
    const float response = smoothResponse(proximity);
    if (previousObstacleId == obstacle.id) {
      previousObstacleActive = true;
    }
    const float normalSpeed = glm::dot(velocity, sample.normal);
    const float approachSpeed = std::max(-normalSpeed, 0.0f);
    float normalStrength = obstacle.strength * response;
    if (approachSpeed > 0.0f) {
      normalStrength += approachSpeed * obstacle.damping;
    }
    result += sample.normal * normalStrength;

    if (approachSpeed <= kEpsilon || obstacle.tangentStrength <= 0.0f) {
      return;
    }

    const float priority = sample.signedDistance /
                           std::max(obstacle.influenceDistance, kEpsilon);
    glm::vec3 tangent(0.0f);
    if (previousObstacleId == obstacle.id) {
      tangent = previousTangent -
                sample.normal * glm::dot(previousTangent, sample.normal);
      tangent = safeNormal(tangent, glm::vec3(0.0f));
    }
    if (glm::length2(tangent) <= kEpsilon) {
      const glm::vec3 tangentialVelocity =
          velocity - sample.normal * normalSpeed;
      tangent = safeNormal(
          tangentialVelocity,
          deterministicTangent(sample.normal, stableId, obstacle.id));
    }

    const float speed2 = glm::length2(velocity);
    const float speed = speed2 > kEpsilon ? std::sqrt(speed2) : 0.0f;
    const float approachRatio =
        speed > kEpsilon ? std::clamp(approachSpeed / speed, 0.0f, 1.0f) : 1.0f;
    const float tangentResponse = response * (0.25f + 0.75f * approachRatio);
    result += tangent * (obstacle.tangentStrength * tangentResponse);

    if (priority < memoryPriority) {
      memoryPriority = priority;
      memory.obstacleId = obstacle.id;
      memory.tangent = tangent;
      selectedObstacle = true;
    }
  };

  if (groundEnabled_) {
    // GroundPlaneは既存の水平面専用経路を維持する。追加Obstacleが空でも
    // 全個体へQuaternion変換を課さないためのhot-pathである。
    SurfaceSample groundSample;
    groundSample.active = true;
    groundSample.signedDistance = position.y - groundPlane_.position.y;
    groundSample.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    accumulate(groundPlane_, groundSample, steering);
  }
  for (const boids::Obstacle &obstacle : obstacles_) {
    accumulate(obstacle, sampleSurface(obstacle, position), steering);
  }
  if (!selectedObstacle && !previousObstacleActive) {
    memory.obstacleId = -1;
    memory.tangent = glm::vec3(0.0f);
  } else if (!selectedObstacle) {
    memory.obstacleId = previousObstacleId;
    memory.tangent = previousTangent;
  }
  return steering;
}

glm::vec3 SteeringEnvironment::computeSteering(
    const glm::vec3 &position, const glm::vec3 &velocity, uint32_t stableId,
    ObstacleAvoidanceMemory &memory) const {
  return computeGuideSteering(position) +
         computeObstacleSteering(position, velocity, stableId, memory);
}

void SteeringEnvironment::resolvePenetration(glm::vec3 &position,
                                             glm::vec3 &velocity) const {
  const auto resolve = [&](const SurfaceSample &sample) {
    if (!sample.active || sample.signedDistance >= 0.0f) {
      return;
    }
    position += sample.normal *
                (-sample.signedDistance + kPenetrationEpsilon);
    const float normalSpeed = glm::dot(velocity, sample.normal);
    if (normalSpeed < 0.0f) {
      velocity -= sample.normal * normalSpeed;
    }
  };

  if (groundEnabled_) {
    SurfaceSample groundSample;
    groundSample.active = true;
    groundSample.signedDistance = position.y - groundPlane_.position.y;
    groundSample.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    resolve(groundSample);
  }
  for (const boids::Obstacle &obstacle : obstacles_) {
    if (obstacle.enabled) {
      resolve(sampleSurface(obstacle, position));
    }
  }
}

float *SteeringEnvironment::resizeGuideInput(std::size_t count) {
  guideInput_.resize(count * kGuideRecordFloats);
  return guideInput_.empty() ? nullptr : guideInput_.data();
}

float *SteeringEnvironment::resizeObstacleInput(std::size_t count) {
  obstacleInput_.resize(count * kObstacleRecordFloats);
  return obstacleInput_.empty() ? nullptr : obstacleInput_.data();
}

bool SteeringEnvironment::commitGuideInput(std::size_t count) {
  if (guideInput_.size() < count * kGuideRecordFloats) {
    return false;
  }
  std::vector<boids::Guide> guides;
  guides.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const float *record = guideInput_.data() + i * kGuideRecordFloats;
    boids::Guide guide;
    guide.enabled = record[0] != 0.0f;
    guide.target = glm::vec3(record[1], record[2], record[3]);
    guide.freeRadius = record[4];
    guide.responseDistance = record[5];
    guide.strength = record[6];
    guides.push_back(guide);
  }
  setGuides(guides);
  return true;
}

bool SteeringEnvironment::commitObstacleInput(std::size_t count) {
  if (obstacleInput_.size() < count * kObstacleRecordFloats) {
    return false;
  }
  std::vector<boids::Obstacle> obstacles;
  obstacles.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const float *record = obstacleInput_.data() + i * kObstacleRecordFloats;
    const int shapeValue = static_cast<int>(record[2]);
    if (shapeValue < static_cast<int>(boids::ObstacleShape::Plane) ||
        shapeValue > static_cast<int>(boids::ObstacleShape::Box)) {
      return false;
    }
    boids::Obstacle obstacle;
    obstacle.enabled = record[0] != 0.0f;
    obstacle.id = static_cast<int32_t>(record[1]);
    obstacle.shape = static_cast<boids::ObstacleShape>(shapeValue);
    obstacle.position = glm::vec3(record[3], record[4], record[5]);
    obstacle.rotation = glm::quat(record[9], record[6], record[7], record[8]);
    obstacle.size = glm::vec3(record[10], record[11], record[12]);
    obstacle.influenceDistance = record[13];
    obstacle.strength = record[14];
    obstacle.damping = record[15];
    obstacle.tangentStrength = record[16];
    obstacles.push_back(obstacle);
  }
  setObstacles(obstacles);
  return true;
}
