#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace {
// Cohesion memory slots per boid.
constexpr int kNeighborSlots = 16;

std::unordered_map<std::string, int> speciesNameToIndex;

float interactionRadius(const SpeciesParams &params) {
  return std::max({params.cohesionRange, params.alignmentRange, params.separationRange});
}

glm::vec3 randomUnitVector(std::mt19937 &rng) {
  std::normal_distribution<float> dist(0.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  const float len = glm::length(v);
  if (len < 1e-6f) {
    return glm::vec3(0.0f, 0.0f, 1.0f);
  }
  return v / len;
}
}

std::vector<SpeciesParams> globalSpeciesParams;

namespace {
SpeciesParams fallbackParamsForId(int speciesId) {
  SpeciesParams params;
  params.speciesId = speciesId;
  params.count = 0;
  return params;
}

SpeciesParams paramsForId(int speciesId) {
  if (speciesId >= 0 && speciesId < static_cast<int>(globalSpeciesParams.size())) {
    const auto &candidate = globalSpeciesParams[speciesId];
    if (!candidate.species.empty() || candidate.count != 0) {
      return candidate;
    }
  }
  return fallbackParamsForId(speciesId);
}
}

BoidTree::BoidTree() {
  buf.reserveAll(1024);
}

BoidTree::~BoidTree() = default;

void BoidTree::setFlockSize(int newSize, float posRange, float velRange) {
  if (newSize < 0) {
    newSize = 0;
  }

  if (globalSpeciesParams.empty()) {
    SpeciesParams params;
    params.species = "Boids";
    params.speciesId = 0;
    params.count = newSize;
    setGlobalSpeciesParams(params);
  } else {
    SpeciesParams params = globalSpeciesParams.front();
    params.count = newSize;
    setGlobalSpeciesParams(params);
  }

  std::vector<SpeciesParams> list;
  list.reserve(globalSpeciesParams.size());
  for (const auto &params : globalSpeciesParams) {
    if (!params.species.empty() && params.count > 0) {
      list.push_back(params);
    }
  }

  if (!list.empty()) {
    initializeBoids(list, posRange, velRange);
  }
}

void BoidTree::initializeBoids(const std::vector<SpeciesParams> &speciesParamsList,
                 float posRange, float velRange) {
  std::vector<SpeciesParams> filtered;
  filtered.reserve(speciesParamsList.size());
  for (const auto &params : speciesParamsList) {
    if (params.count <= 0) {
      continue;
    }
    filtered.push_back(params);
    setGlobalSpeciesParams(params);
  }

  int totalBoids = 0;
  for (const auto &params : filtered) {
    totalBoids += params.count;
  }

  buf.resizeAll(totalBoids);
  ensureProxyStorage();
  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<float> posDist(-posRange, posRange);
  std::uniform_real_distribution<float> speedDist(0.0f, velRange);

  int index = 0;
  for (const auto &params : filtered) {
    for (int i = 0; i < params.count; ++i, ++index) {
      buf.positions[index] = glm::vec3(posDist(rng), posDist(rng), posDist(rng));
      const glm::vec3 dir = randomUnitVector(rng);
      buf.velocities[index] = dir * speedDist(rng);
      buf.accelerations[index] = glm::vec3(0.0f);
      buf.orientations[index] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      buf.ids[index] = index;
      buf.stresses[index] = 0.0f;
      buf.speciesIds[index] = params.speciesId;
      buf.isAttracting[index] = 0;
      buf.attractTimers[index] = 0.0f;
      buf.predatorInfluences[index] = glm::vec3(0.0f);
      buf.predatorTargetIndices[index] = -1;
      buf.predatorTargetTimers[index] = 0.0f;
      buf.boidCohesionMemories[index].assign(kNeighborSlots, -1.0f);
      buf.boidActiveNeighbors[index].reset();
      buf.boidNeighborSlots[index].assign(kNeighborSlots, -1);
    }
  }

  initializeBoidMemories(filtered);
  rebuildBVH();
  frameCount = 0;
}

void BoidTree::initializeBoidMemories(const std::vector<SpeciesParams> &speciesParamsList) {
  (void)speciesParamsList;
  for (std::size_t i = 0; i < buf.boidCohesionMemories.size(); ++i) {
    buf.boidCohesionMemories[i].assign(kNeighborSlots, -1.0f);
    buf.boidActiveNeighbors[i].reset();
    buf.boidNeighborSlots[i].assign(kNeighborSlots, -1);
  }
  buf.cohesionMemories.clear();
}

void BoidTree::build(int /*maxPerUnit*/, int /*level*/) {
  rebuildBVH();
}

void BoidTree::update(float dt) {
  if (buf.positions.empty()) {
    return;
  }

  if (!(dt > 0.0f)) {
    dt = 1.0f / 60.0f;
  }

  ensureProxyStorage();
  processRange(0, static_cast<int>(buf.positions.size()), dt);
  integrateRange(0, static_cast<int>(buf.positions.size()), dt);
  updateBVH();
  ++frameCount;
}

uintptr_t BoidTree::getPositionsPtr() {
  return reinterpret_cast<uintptr_t>(buf.positions.data());
}

uintptr_t BoidTree::getVelocitiesPtr() {
  return reinterpret_cast<uintptr_t>(buf.velocities.data());
}

uintptr_t BoidTree::getOrientationsPtr() {
  return reinterpret_cast<uintptr_t>(buf.orientations.data());
}

int BoidTree::getBoidCount() const {
  return static_cast<int>(buf.positions.size());
}

std::unordered_map<int, int> BoidTree::collectBoidUnitMapping() {
  std::unordered_map<int, int> mapping;
  mapping.reserve(buf.ids.size());
  for (std::size_t i = 0; i < buf.ids.size(); ++i) {
    mapping.emplace(buf.ids[i], buf.speciesIds[i]);
  }
  return mapping;
}

SpeciesParams BoidTree::getGlobalSpeciesParams(std::string species) {
  const auto it = speciesNameToIndex.find(species);
  if (it != speciesNameToIndex.end()) {
    const int id = it->second;
    if (id >= 0 && id < static_cast<int>(globalSpeciesParams.size())) {
      return globalSpeciesParams[id];
    }
  }
  return SpeciesParams{};
}

void BoidTree::setGlobalSpeciesParams(const SpeciesParams &params) {
  if (params.speciesId < 0) {
    return;
  }

  if (params.speciesId >= static_cast<int>(globalSpeciesParams.size())) {
    globalSpeciesParams.resize(params.speciesId + 1);
  }

  globalSpeciesParams[params.speciesId] = params;
  speciesNameToIndex[params.species] = params.speciesId;

  maxInteractionRadius = 0.0f;
  for (const auto &sp : globalSpeciesParams) {
    if (sp.species.empty() && sp.count == 0) {
      continue;
    }
    const float maxRange = interactionRadius(sp);
    maxInteractionRadius = std::max(maxInteractionRadius, maxRange);
  }
  if (maxInteractionRadius <= 0.0f) {
    maxInteractionRadius = 10.0f;
  }
}

void BoidTree::rebuildBVH() {
  bvh.clear();
  ensureProxyStorage();
  for (std::size_t i = 0; i < buf.positions.size(); ++i) {
    const SpeciesParams params = paramsForId(buf.speciesIds[i]);
    const float radius = std::max(interactionRadius(params), 0.1f);
    const glm::vec3 extent(radius);
    const glm::vec3 padding = computeFatMargin(buf.speciesIds[i]);
    const glm::vec3 min = buf.positions[i] - extent;
    const glm::vec3 max = buf.positions[i] + extent;
    proxyIds[i] = bvh.createProxy(min - padding, max + padding, static_cast<int>(i));
  }
}

void BoidTree::ensureProxyStorage() {
  proxyIds.resize(buf.positions.size(), -1);
}

void BoidTree::updateBVH() {
  ensureProxyStorage();
  for (std::size_t i = 0; i < buf.positions.size(); ++i) {
    const SpeciesParams params = paramsForId(buf.speciesIds[i]);
    const float radius = std::max(interactionRadius(params), 0.1f);
    const glm::vec3 extent(radius);
    const glm::vec3 padding = computeFatMargin(buf.speciesIds[i]);
    const glm::vec3 min = buf.positions[i] - extent;
    const glm::vec3 max = buf.positions[i] + extent;
    if (i >= proxyIds.size() || proxyIds[i] < 0) {
      proxyIds[i] = bvh.createProxy(min - padding, max + padding, static_cast<int>(i));
    } else {
      bvh.updateProxy(proxyIds[i], min, max, padding);
    }
  }
}

void BoidTree::processRange(int start, int end, float /*dt*/) {
  for (int i = start; i < end; ++i) {
    const int speciesId = (i < static_cast<int>(buf.speciesIds.size())) ? buf.speciesIds[i] : 0;
    const SpeciesParams params = paramsForId(speciesId);

    const glm::vec3 pos = buf.positions[i];
    const glm::vec3 vel = buf.velocities[i];
    const float maxRange = interactionRadius(params);
    const float separationR2 = params.separationRange * params.separationRange;
    const float alignmentR2 = params.alignmentRange * params.alignmentRange;
    const float cohesionR2 = params.cohesionRange * params.cohesionRange;
    const float cosHalfFov = std::cos(glm::radians(params.fieldOfViewDeg * 0.5f));

    glm::vec3 separationForce(0.0f);
    glm::vec3 alignmentForce(0.0f);
    glm::vec3 cohesionAccumulator(0.0f);
    int alignmentCount = 0;
    int cohesionCount = 0;
    int neighborSamples = 0;

    std::array<int, kNeighborSlots> neighborIndices{};
    neighborIndices.fill(-1);
    int storedNeighbors = 0;

  const glm::vec3 queryMin = pos - glm::vec3(maxRange);
  const glm::vec3 queryMax = pos + glm::vec3(maxRange);

    bvh.query(queryMin, queryMax, [&](int otherIndex) {
      if (otherIndex == i) {
        return true;
      }
      const glm::vec3 offset = buf.positions[otherIndex] - pos;
      const float dist2 = glm::length2(offset);
      if (dist2 < 1e-8f) {
        return true;
      }

      const float dist = std::sqrt(dist2);
      const glm::vec3 dir = offset / dist;

      if (glm::length2(vel) > 1e-8f) {
        const float view = glm::dot(glm::normalize(vel), dir);
        if (view < cosHalfFov) {
          return true;
        }
      }

      if (dist2 <= separationR2 && params.separation > 0.0f) {
        separationForce -= dir / std::max(dist, 1e-4f);
      }

      if (dist2 <= alignmentR2 && params.alignment > 0.0f) {
        alignmentForce += buf.velocities[otherIndex];
        ++alignmentCount;
      }

      if (dist2 <= cohesionR2 && params.cohesion > 0.0f) {
        cohesionAccumulator += buf.positions[otherIndex];
        ++cohesionCount;
      }

      if (storedNeighbors < kNeighborSlots) {
        neighborIndices[storedNeighbors++] = otherIndex;
      }

      ++neighborSamples;
      if (params.maxNeighbors > 0 && neighborSamples >= params.maxNeighbors) {
        return false;
      }
      return true;
    });

    glm::vec3 acceleration(0.0f);
    if (params.separation > 0.0f) {
      acceleration += separationForce * params.separation;
    }

    if (alignmentCount > 0 && params.alignment > 0.0f) {
      const glm::vec3 averageVelocity = alignmentForce / static_cast<float>(alignmentCount);
      acceleration += (averageVelocity - vel) * params.alignment;
    }

    if (cohesionCount > 0 && params.cohesion > 0.0f) {
      const glm::vec3 center = cohesionAccumulator / static_cast<float>(cohesionCount);
      acceleration += (center - pos) * params.cohesion * 0.01f;
    }

    buf.accelerations[i] = acceleration;

    auto &active = buf.boidActiveNeighbors[i];
    auto &slots = buf.boidNeighborSlots[i];
    auto &memories = buf.boidCohesionMemories[i];
    active.reset();
    for (int s = 0; s < kNeighborSlots; ++s) {
      if (s < storedNeighbors && neighborIndices[s] >= 0) {
        active.set(s);
        slots[s] = neighborIndices[s];
        memories[s] = 0.0f;
      } else {
        slots[s] = -1;
        memories[s] = -1.0f;
      }
    }
  }
}

void BoidTree::integrateRange(int start, int end, float dt) {
  for (int i = start; i < end; ++i) {
    const int speciesId = (i < static_cast<int>(buf.speciesIds.size())) ? buf.speciesIds[i] : 0;
    const SpeciesParams params = paramsForId(speciesId);

    glm::vec3 velocity = buf.velocities[i] + buf.accelerations[i] * dt;
    const float speed = glm::length(velocity);

    if (speed > params.maxSpeed && speed > 1e-6f) {
      velocity = (velocity / speed) * params.maxSpeed;
    } else if (speed < params.minSpeed && speed > 1e-6f) {
      velocity = (velocity / speed) * params.minSpeed;
    }

    buf.velocities[i] = velocity;
    buf.positions[i] += velocity * dt;
    buf.accelerations[i] = glm::vec3(0.0f);

    if (glm::length2(velocity) > 1e-8f) {
      const glm::vec3 forward = glm::normalize(velocity);
      const glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
      buf.orientations[i] = glm::quatLookAt(forward, up);
    }
  }
}

glm::vec3 BoidTree::computeFatMargin(int speciesId) const {
  const SpeciesParams params = paramsForId(speciesId);
  const float speedPadding = std::max(params.maxSpeed * 8.0f, fatMargin.x);
  return glm::vec3(speedPadding);
}
