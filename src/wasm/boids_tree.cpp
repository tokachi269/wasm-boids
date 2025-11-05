#include "boids_tree.h"

#include "boid_unit.h"
#include "platform_utils.h"
#include "species_params.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

std::vector<SpeciesParams> globalSpeciesParams;

namespace {

// 速度ベクトルから Z+ 向きを基準とした姿勢クォータニオンを生成。
// 静止状態では単位クォータニオンを返す。
glm::quat orientationFromVelocity(const glm::vec3 &velocity) {
  const float speedSq = glm::dot(velocity, velocity);
  if (speedSq < 1e-8f) {
    return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  }

  const glm::vec3 forward = velocity * (1.0f / glm::sqrt(speedSq));
  glm::vec3 up(0.0f, 1.0f, 0.0f);
  if (std::fabs(glm::dot(forward, up)) > 0.99f) {
    up = glm::vec3(1.0f, 0.0f, 0.0f);
  }

  glm::vec3 right = glm::cross(up, forward);
  const float rightLenSq = glm::dot(right, right);
  if (rightLenSq < 1e-8f) {
    right = glm::vec3(1.0f, 0.0f, 0.0f);
  } else {
    right *= 1.0f / glm::sqrt(rightLenSq);
  }
  const glm::vec3 correctedUp = glm::cross(forward, right);
  const glm::mat3 basis(right, correctedUp, forward);
  return glm::quat_cast(basis);
}

} // namespace

BoidTree &BoidTree::instance() {
  static BoidTree instance;
  return instance;
}

BoidTree::BoidTree() : lbvhIndex_(32) {}

BoidTree::~BoidTree() = default;

void BoidTree::setRenderPointersToReadBuffers() {
  renderPositionsPtr_ = buf.positions.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(buf.positions.data());
  renderVelocitiesPtr_ = buf.velocities.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(buf.velocities.data());
  renderOrientationsPtr_ =
      buf.orientations.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.orientations.data());
}

void BoidTree::setRenderPointersToWriteBuffers() {
  renderPositionsPtr_ = buf.positionsWrite.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(buf.positionsWrite.data());
  renderVelocitiesPtr_ = buf.velocitiesWrite.empty()
                            ? 0
                            : reinterpret_cast<uintptr_t>(buf.velocitiesWrite.data());
  renderOrientationsPtr_ =
      buf.orientationsWrite.empty()
          ? 0
          : reinterpret_cast<uintptr_t>(buf.orientationsWrite.data());
}

void BoidTree::build(int maxPerUnit) {
  maxBoidsPerUnit = maxPerUnit;
  lbvhIndex_.build(buf);
  setRenderPointersToReadBuffers();
}

SpatialIndex &BoidTree::spatialIndex() { return lbvhIndex_; }

const SpatialIndex &BoidTree::spatialIndex() const { return lbvhIndex_; }

void BoidTree::forEachLeaf(const LeafVisitor &visitor) const {
  lbvhIndex_.forEachLeaf(visitor);
}

void BoidTree::forEachLeafIntersectingSphere(const glm::vec3 &center,
                                             float radius,
                                             const LeafVisitor &visitor) const {
  lbvhIndex_.forEachLeafIntersectingSphere(center, radius, visitor);
}

void BoidTree::update(float dt) {
  // dt を制限して数値爆発を避ける
  const float clampedDt = std::clamp(dt, 0.0f, 0.1f) * 5.0f;

  const int count = static_cast<int>(buf.positions.size());
  if (count == 0 || clampedDt <= 0.0f) {
    setRenderPointersToReadBuffers();
    lbvhIndex_.build(buf);
    frameCount++;
    return;
  }

  setRenderPointersToReadBuffers();
  lbvhIndex_.resetQueryStats();
  computeBoidInteractionsRange(buf, lbvhIndex_, 0, count, clampedDt);
  lastQueryStats_ = lbvhIndex_.consumeQueryStats();

  setRenderPointersToWriteBuffers();
  updateBoidKinematicsRange(buf, 0, count, clampedDt);
  buf.swapReadWrite();
  setRenderPointersToReadBuffers();
  lbvhIndex_.build(buf);
  frameCount++;
}

void BoidTree::initializeBoids(
    const std::vector<SpeciesParams> &speciesParamsList, float posRange,
    float velRange) {
  globalSpeciesParams = speciesParamsList;

  int totalCount = 0;
  for (const auto &species : globalSpeciesParams) {
    totalCount += std::max(species.count, 0);
  }

  buf.reserveAll(totalCount);
  buf.resizeAll(totalCount);

  std::fill(buf.accelerations.begin(), buf.accelerations.end(), glm::vec3(0.0f));
  std::fill(buf.predatorInfluences.begin(), buf.predatorInfluences.end(),
            glm::vec3(0.0f));
  std::fill(buf.predatorThreats.begin(), buf.predatorThreats.end(), 0.0f);
  std::fill(buf.predatorTargetIndices.begin(), buf.predatorTargetIndices.end(),
            -1);
  std::fill(buf.predatorTargetTimers.begin(), buf.predatorTargetTimers.end(),
            0.0f);
  std::fill(buf.stresses.begin(), buf.stresses.end(), 0.0f);
  std::fill(buf.isAttracting.begin(), buf.isAttracting.end(), 0);
  std::fill(buf.attractTimers.begin(), buf.attractTimers.end(), 0.0f);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> posDist(-posRange, posRange);
  std::uniform_real_distribution<float> velDist(-velRange, velRange);

  int offset = 0;
  for (std::size_t speciesId = 0; speciesId < globalSpeciesParams.size();
       ++speciesId) {
    const auto &species = globalSpeciesParams[speciesId];
    const int countForSpecies = std::max(species.count, 0);
    for (int i = 0; i < countForSpecies; ++i, ++offset) {
      const glm::vec3 position(posDist(gen), posDist(gen), posDist(gen));
      const glm::vec3 velocity(velDist(gen), velDist(gen), velDist(gen));
      const glm::quat orientation = orientationFromVelocity(velocity);

      buf.positions[offset] = position;
      buf.velocities[offset] = velocity;
      buf.ids[offset] = offset;
      buf.speciesIds[offset] = static_cast<int>(speciesId);
      buf.orientations[offset] = orientation;
    }
  }

  buf.syncWriteFromRead();
  setRenderPointersToReadBuffers();
  initializeBoidMemories(globalSpeciesParams);
  lbvhIndex_.build(buf);
  frameCount = 0;
}

void BoidTree::initializeBoidMemories(
    const std::vector<SpeciesParams> &speciesParamsList) {
  const int totalCount = static_cast<int>(buf.positions.size());

  if (totalCount == 0) {
    buf.boidCohesionMemories.clear();
    buf.boidNeighborMasks.clear();
    buf.boidNeighborIndices.clear();
    return;
  }

  buf.boidCohesionMemories.resize(totalCount);
  buf.boidNeighborMasks.resize(totalCount, 0);
  buf.boidNeighborIndices.resize(totalCount);

  if (speciesParamsList.empty()) {
    logger::log("Warning: speciesParamsList is empty, using default neighbor slots");
    for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
      buf.boidCohesionMemories[boidIndex].assign(4, 0.0f);
      buf.boidNeighborMasks[boidIndex] = 0;
      buf.boidNeighborIndices[boidIndex].fill(-1);
    }
    return;
  }

  for (int boidIndex = 0; boidIndex < totalCount; ++boidIndex) {
    const int speciesId =
        (boidIndex < static_cast<int>(buf.speciesIds.size()))
            ? buf.speciesIds[boidIndex]
            : -1;
    if (speciesId < 0 ||
        speciesId >= static_cast<int>(speciesParamsList.size())) {
      buf.boidCohesionMemories[boidIndex].assign(4, 0.0f);
      buf.boidNeighborMasks[boidIndex] = 0;
      buf.boidNeighborIndices[boidIndex].fill(-1);
      continue;
    }

    const auto &species = speciesParamsList[speciesId];
    const int slotCount = std::clamp(species.maxNeighbors, 1, 16);
    buf.boidCohesionMemories[boidIndex].assign(slotCount, 0.0f);
    buf.boidNeighborMasks[boidIndex] = 0;
    buf.boidNeighborIndices[boidIndex].fill(-1);
  }
}

void BoidTree::setFlockSize(int newSize, float posRange, float velRange) {
  newSize = std::max(0, newSize);
  const int current = static_cast<int>(buf.positions.size());
  if (newSize == current) {
    return;
  }

  if (newSize < current) {
    buf.positions.resize(newSize);
    buf.positionsWrite.resize(newSize);
    buf.velocities.resize(newSize);
    buf.velocitiesWrite.resize(newSize);
    buf.accelerations.resize(newSize);
    buf.ids.resize(newSize);
    buf.stresses.resize(newSize);
    buf.speciesIds.resize(newSize);
    buf.isAttracting.resize(newSize);
    buf.attractTimers.resize(newSize);
    buf.orientations.resize(newSize);
    buf.orientationsWrite.resize(newSize);
    buf.predatorTargetIndices.resize(newSize, -1);
    buf.predatorTargetTimers.resize(newSize, 0.0f);
    buf.predatorInfluences.resize(newSize);
    buf.predatorThreats.resize(newSize);
    buf.boidCohesionMemories.resize(newSize);
    buf.boidNeighborMasks.resize(newSize, 0);
    buf.boidNeighborIndices.resize(newSize);
    for (int i = newSize; i < current; ++i) {
      buf.cohesionMemories.erase(i);
    }
  } else {
    const int addCount = newSize - current;
    buf.reserveAll(newSize);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int k = 0; k < addCount; ++k) {
      const int index = current + k;
      const glm::vec3 position(posDist(gen), posDist(gen), posDist(gen));
      const glm::vec3 velocity(velDist(gen), velDist(gen), velDist(gen));
      const glm::quat orientation = orientationFromVelocity(velocity);

      buf.positions.push_back(position);
      buf.positionsWrite.push_back(position);
      buf.velocities.push_back(velocity);
      buf.velocitiesWrite.push_back(velocity);
      buf.accelerations.push_back(glm::vec3(0.0f));
      buf.ids.push_back(index);
      buf.stresses.push_back(0.0f);
      buf.speciesIds.push_back(0);
      buf.isAttracting.push_back(0);
      buf.attractTimers.push_back(0.0f);
  buf.orientations.push_back(orientation);
  buf.orientationsWrite.push_back(orientation);
      buf.predatorTargetIndices.push_back(-1);
      buf.predatorTargetTimers.push_back(0.0f);
      buf.predatorInfluences.push_back(glm::vec3(0.0f));
      buf.predatorThreats.push_back(0.0f);
      buf.boidCohesionMemories.emplace_back();
      buf.boidNeighborMasks.push_back(0);
      buf.boidNeighborIndices.emplace_back();
      buf.boidNeighborIndices.back().fill(-1);
      buf.cohesionMemories[index] = std::unordered_map<int, float>();
    }
  }

  initializeBoidMemories(globalSpeciesParams);
  buf.syncWriteFromRead();
  setRenderPointersToReadBuffers();
  lbvhIndex_.build(buf);
}

uintptr_t BoidTree::getPositionsPtr() {
  if (!renderPositionsPtr_ && !buf.positions.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderPositionsPtr_;
}

uintptr_t BoidTree::getVelocitiesPtr() {
  if (!renderVelocitiesPtr_ && !buf.velocities.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderVelocitiesPtr_;
}

uintptr_t BoidTree::getOrientationsPtr() {
  if (!renderOrientationsPtr_ && !buf.orientations.empty()) {
    setRenderPointersToReadBuffers();
  }
  return renderOrientationsPtr_;
}

int BoidTree::getBoidCount() const {
  return static_cast<int>(buf.positions.size());
}

std::unordered_map<int, int> BoidTree::collectBoidUnitMapping() {
  std::unordered_map<int, int> mapping;
  const int count = static_cast<int>(buf.positions.size());
  mapping.reserve(count);
  for (int i = 0; i < count; ++i) {
    mapping.emplace(i, i);
  }
  return mapping;
}

SpeciesParams BoidTree::getGlobalSpeciesParams(std::string species) {
  const auto it = std::find_if(
      globalSpeciesParams.begin(), globalSpeciesParams.end(),
      [&species](const SpeciesParams &p) { return p.species == species; });
  if (it != globalSpeciesParams.end()) {
    return *it;
  }
  return {};
}

void BoidTree::setGlobalSpeciesParams(const SpeciesParams &params) {
  const auto it = std::find_if(globalSpeciesParams.begin(),
                               globalSpeciesParams.end(),
                               [&params](const SpeciesParams &p) {
                                 return p.species == params.species;
                               });
  if (it != globalSpeciesParams.end()) {
    *it = params;
  } else {
    globalSpeciesParams.push_back(params);
  }

  initializeBoidMemories(globalSpeciesParams);
  lbvhIndex_.build(buf);
}
