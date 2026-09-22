#include "steering_environment.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <vector>

namespace {

bool near(float actual, float expected, float tolerance = 1e-4f) {
  return std::abs(actual - expected) <= tolerance;
}

bool nearVec(const glm::vec3 &actual, const glm::vec3 &expected,
             float tolerance = 1e-4f) {
  return near(actual.x, expected.x, tolerance) &&
         near(actual.y, expected.y, tolerance) &&
         near(actual.z, expected.z, tolerance);
}

int fail(const char *message) {
  std::cerr << message << '\n';
  return 1;
}

} // namespace

int main() {
  SteeringEnvironment environment;
  environment.configureGroundPlane(false, 0.0f, 2.0f, 18.0f, 8.0f);

  boids::Guide guide;
  guide.target = glm::vec3(0.0f);
  guide.freeRadius = 1.0f;
  guide.responseDistance = 2.0f;
  guide.strength = 4.0f;
  environment.setGuides({guide});

  ObstacleAvoidanceMemory memory;
  const glm::vec3 insideGuide = environment.computeSteering(
      glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f), 1, memory);
  if (!nearVec(insideGuide, glm::vec3(0.0f))) {
    return fail("Guide applied steering inside freeRadius");
  }
  const glm::vec3 outsideGuide = environment.computeSteering(
      glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(0.0f), 1, memory);
  if (!nearVec(outsideGuide, glm::vec3(-4.0f, 0.0f, 0.0f))) {
    return fail("Guide response did not reach configured strength");
  }

  float *packedGuide = environment.resizeGuideInput(1);
  const float packedGuideValues[SteeringEnvironment::kGuideRecordFloats] = {
      1.0f, 5.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f};
  std::copy(std::begin(packedGuideValues), std::end(packedGuideValues),
            packedGuide);
  if (!environment.commitGuideInput(1)) {
    return fail("Packed Guide input was rejected");
  }
  const glm::vec3 packedGuideSteering = environment.computeSteering(
      glm::vec3(0.0f), glm::vec3(0.0f), 1, memory);
  if (!nearVec(packedGuideSteering, glm::vec3(3.0f, 0.0f, 0.0f))) {
    return fail("Packed Guide input was decoded incorrectly");
  }

  environment.setGuides({});
  const struct ShapeCase {
    boids::ObstacleShape shape;
    glm::vec3 size;
    glm::vec3 point;
    float distance;
    glm::vec3 normal;
  } cases[] = {
      {boids::ObstacleShape::Plane, glm::vec3(1.0f),
       glm::vec3(0.0f, 2.0f, 0.0f), 2.0f, glm::vec3(0.0f, 1.0f, 0.0f)},
      {boids::ObstacleShape::Sphere, glm::vec3(1.0f),
       glm::vec3(3.0f, 0.0f, 0.0f), 2.0f, glm::vec3(1.0f, 0.0f, 0.0f)},
      {boids::ObstacleShape::Capsule, glm::vec3(1.0f, 2.0f, 1.0f),
       glm::vec3(2.0f, 0.0f, 0.0f), 1.0f, glm::vec3(1.0f, 0.0f, 0.0f)},
      {boids::ObstacleShape::Box, glm::vec3(1.0f),
       glm::vec3(3.0f, 0.0f, 0.0f), 2.0f, glm::vec3(1.0f, 0.0f, 0.0f)},
  };

  for (const ShapeCase &shapeCase : cases) {
    boids::Obstacle obstacle;
    obstacle.shape = shapeCase.shape;
    obstacle.size = shapeCase.size;
    const auto sample = environment.sampleSurface(obstacle, shapeCase.point);
    if (!sample.active || !near(sample.signedDistance, shapeCase.distance) ||
        !nearVec(sample.normal, shapeCase.normal)) {
      return fail("Obstacle signed distance or normal is incorrect");
    }
  }

  boids::Obstacle sphere;
  sphere.id = 42;
  sphere.shape = boids::ObstacleShape::Sphere;
  sphere.size = glm::vec3(1.0f);
  sphere.influenceDistance = 2.0f;
  sphere.strength = 2.0f;
  sphere.damping = 2.0f;
  sphere.tangentStrength = 3.0f;
  environment.setObstacles({sphere});

  float *packedObstacle = environment.resizeObstacleInput(1);
  const float packedObstacleValues[
      SteeringEnvironment::kObstacleRecordFloats] = {
      1.0f, 42.0f, 1.0f,
      0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f,
      1.0f, 1.0f, 1.0f,
      2.0f, 2.0f, 2.0f, 3.0f};
  std::copy(std::begin(packedObstacleValues),
            std::end(packedObstacleValues), packedObstacle);
  if (!environment.commitObstacleInput(1)) {
    return fail("Packed Obstacle input was rejected");
  }

  memory = {};
  const glm::vec3 firstAvoidance = environment.computeSteering(
      glm::vec3(2.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), 7,
      memory);
  if (firstAvoidance.x <= 0.0f ||
      glm::dot(glm::vec3(0.0f, firstAvoidance.y, firstAvoidance.z),
               glm::vec3(0.0f, firstAvoidance.y, firstAvoidance.z)) <=
          1e-6f ||
      memory.obstacleId != 42) {
    std::cerr << "avoidance=" << firstAvoidance.x << ',' << firstAvoidance.y
              << ',' << firstAvoidance.z << " memory=" << memory.obstacleId
              << " tangent=" << memory.tangent.x << ',' << memory.tangent.y
              << ',' << memory.tangent.z << '\n';
    return fail("Obstacle avoidance did not push away and choose a tangent");
  }
  const glm::vec3 retainedTangent = memory.tangent;
  environment.computeSteering(glm::vec3(1.8f, 0.0f, 0.0f),
                              glm::vec3(-1.0f, 0.0f, 0.0f), 7, memory);
  if (glm::dot(retainedTangent, memory.tangent) <= 0.0f) {
    return fail("Obstacle avoidance side was not retained");
  }
  environment.computeSteering(glm::vec3(1.8f, 0.0f, 0.0f),
                              glm::vec3(1.0f, 0.0f, 0.0f), 7, memory);
  if (memory.obstacleId != 42 ||
      glm::dot(retainedTangent, memory.tangent) <= 0.0f) {
    return fail("Obstacle avoidance side was cleared before leaving influence");
  }

  glm::vec3 penetratingPosition(0.5f, 0.0f, 0.0f);
  glm::vec3 penetratingVelocity(-1.0f, 0.0f, 0.0f);
  environment.resolvePenetration(penetratingPosition, penetratingVelocity);
  if (penetratingPosition.x < 1.0f || penetratingVelocity.x < 0.0f) {
    return fail("Penetration correction did not move outside the sphere");
  }

  return 0;
}
