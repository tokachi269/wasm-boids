#define GLM_ENABLE_EXPERIMENTAL

#include "boid_unit.h"

#include "boids_buffers.h"
#include "lbvh_index.h"
#include "simulation_tuning.h"
#include "species_params.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>

extern std::vector<SpeciesParams> globalSpeciesParams;

namespace {

constexpr float kEpsilon = 1e-6f;
constexpr float kMinDirLen2 = 1e-12f;
constexpr int kMaxNeighborSlots = 16;

inline bool maskTest(uint16_t mask, int slot) {
  return ((mask >> slot) & 1u) != 0;
}

inline void maskSet(uint16_t &mask, int slot) {
  mask |= static_cast<uint16_t>(1u << slot);
}

inline void maskReset(uint16_t &mask, int slot) {
  mask &= static_cast<uint16_t>(~(1u << slot));
}

inline glm::vec3 approxRotate(const glm::vec3 &v, const glm::vec3 &axis,
                              float angle) {
  const float axisLen2 = glm::length2(axis);
  if (axisLen2 < 1e-8f) {
    return v;
  }
  const glm::vec3 normalizedAxis = axis * (1.0f / glm::sqrt(axisLen2));
  return v + angle * glm::cross(normalizedAxis, v);
}

inline glm::quat dirToQuatRollZero(const glm::vec3 &forward) {
  glm::vec3 f = glm::normalize(forward);
  glm::vec3 up(0.0f, 1.0f, 0.0f);
  if (std::fabs(glm::dot(f, up)) > 0.99f) {
    up = glm::vec3(1.0f, 0.0f, 0.0f);
  }
  glm::vec3 right = glm::normalize(glm::cross(up, f));
  up = glm::cross(f, right);
  glm::mat3 basis(right, up, f);
  return glm::quat_cast(basis);
}

inline float easeOut(float t) {
  return t * t * (3.0f - 2.0f * t);
}

inline float smoothStep(float edge0, float edge1, float x) {
  if (edge0 == edge1) {
    return x < edge0 ? 0.0f : 1.0f;
  }
  const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline glm::vec3 safeNormalize(const glm::vec3 &v) {
  const float lenSq = glm::dot(v, v);
  if (lenSq < kMinDirLen2) {
    return glm::vec3(0.0f);
  }
  return v * glm::inversesqrt(lenSq);
}

inline uint32_t nextRandom() {
  static thread_local uint32_t state = 1u;
  state = state * 1103515245u + 12345u;
  return (state >> 16) & 0x7fffu;
}

inline int findFreeSlot(uint16_t mask, int limit) {
  for (int i = 0; i < limit; ++i) {
    if (!maskTest(mask, i)) {
      return i;
    }
  }
  return -1;
}

inline bool containsNeighbor(uint16_t mask,
                             const std::array<int, kMaxNeighborSlots> &slots,
                             int limit, int candidate) {
  for (int i = 0; i < limit; ++i) {
    if (maskTest(mask, i) && slots[i] == candidate) {
      return true;
    }
  }
  return false;
}

float computeMaxPredatorAlertRadius() {
  float radius = 1.0f;
  for (const auto &params : globalSpeciesParams) {
    if (params.isPredator) {
      continue;
    }
    radius = std::max(radius, std::max(params.predatorAlertRadius, 0.0f));
  }
  return radius;
}

} // namespace

void computeBoidInteractionsRange(SoABuffers &buf, const LbvhIndex &index,
                                  int begin, int end, float dt) {
  const int totalCount = static_cast<int>(buf.positions.size());
  if (totalCount == 0) {
    return;
  }

  begin = std::clamp(begin, 0, totalCount);
  end = std::clamp(end, begin, totalCount);
  if (begin >= end) {
    return;
  }

  const float maxPredatorAlertRadius = computeMaxPredatorAlertRadius();

  constexpr int kMaxCandidatePool = kMaxNeighborSlots * 2;
  constexpr int kMaxPredatorCandidates = 64;

  for (int gIdx = begin; gIdx < end; ++gIdx) {
    const int sid = buf.speciesIds[gIdx];
    if (sid < 0 || sid >= static_cast<int>(globalSpeciesParams.size())) {
      continue;
    }
    const SpeciesParams &selfParams = globalSpeciesParams[sid];

    glm::vec3 newAcceleration(0.0f);

    auto &cohesionMemories = buf.boidCohesionMemories[gIdx];
    uint16_t &activeNeighbors = buf.boidNeighborMasks[gIdx];
    auto &neighborIndices = buf.boidNeighborIndices[gIdx];

    const int desiredSlots =
        std::min(kMaxNeighborSlots, std::max(1, selfParams.maxNeighbors));
    if (static_cast<int>(cohesionMemories.size()) < desiredSlots) {
      cohesionMemories.resize(desiredSlots, 0.0f);
    }
    const int slotCount = std::min(desiredSlots, kMaxNeighborSlots);

    for (int slot = slotCount; slot < kMaxNeighborSlots; ++slot) {
      maskReset(activeNeighbors, slot);
      neighborIndices[slot] = -1;
    }

    int activeCount = 0;
    for (int slot = 0; slot < slotCount; ++slot) {
      if (!maskTest(activeNeighbors, slot)) {
        neighborIndices[slot] = -1;
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
        continue;
      }

      const int neighborIdx = neighborIndices[slot];
      bool remove = neighborIdx < 0 || neighborIdx >= totalCount;
      if (!remove && buf.speciesIds[neighborIdx] != sid) {
        remove = true;
      }

      if (!remove && slot < static_cast<int>(cohesionMemories.size())) {
        cohesionMemories[slot] += dt;
        if (cohesionMemories[slot] > selfParams.tau) {
          remove = true;
        }
      }

      if (remove) {
        maskReset(activeNeighbors, slot);
        neighborIndices[slot] = -1;
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
      } else {
        ++activeCount;
      }
    }

    const glm::vec3 pos = buf.positions[gIdx];
    const glm::vec3 vel = buf.velocities[gIdx];
    const float velLen2 = glm::length2(vel);
    glm::vec3 forward = safeNormalize(vel);
    if (glm::length2(forward) < 1e-8f) {
      forward = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    std::array<float, kMaxCandidatePool> candidateDistSquared{};
    std::array<int, kMaxCandidatePool> candidateIndices{};
    int candidateCount = 0;

    std::array<int, kMaxPredatorCandidates> predatorCandidates{};
    int predatorCandidateCount = 0;

    glm::vec3 attractDirSum(0.0f);
    int attractDirCount = 0;

    const float queryRadius = std::max(selfParams.cohesionRange,
                                       std::max(selfParams.separationRange, 0.0f));
    const float viewRangeSq = queryRadius * queryRadius;
    const float halfFovRad =
        glm::radians(std::max(selfParams.fieldOfViewDeg, 0.0f) * 0.5f);
    const float cosHalfFov = std::cos(halfFovRad);

    const float reSq = selfParams.separationRange * selfParams.separationRange;
    const float raSq = selfParams.cohesionRange * selfParams.cohesionRange;
  const int candidateCapacity =
    std::max(1, std::min(kMaxCandidatePool, std::max(slotCount * 2, 4)));

    const int fetched = index.gatherNearest(
        pos, candidateCapacity, queryRadius, candidateIndices.data(),
        candidateDistSquared.data());

    for (int i = 0; i < fetched; ++i) {
      const int neighborIdx = candidateIndices[i];
      const float distSq = candidateDistSquared[i];
      if (neighborIdx == gIdx || neighborIdx < 0 || neighborIdx >= totalCount) {
        continue;
      }
      if (buf.speciesIds[neighborIdx] != sid) {
        continue;
      }
      if (distSq < kEpsilon || distSq >= viewRangeSq) {
        continue;
      }

      const glm::vec3 diff = buf.positions[neighborIdx] - pos;
      const float diffDot = glm::dot(forward, diff);
      const float requiredDot = cosHalfFov * glm::sqrt(distSq);
      if (diffDot < requiredDot) {
        continue;
      }

      if (distSq > reSq && distSq <= raSq) {
        const float dist = glm::sqrt(std::max(distSq, kEpsilon));
        attractDirSum += diff / dist;
        ++attractDirCount;
      }

  if (!containsNeighbor(activeNeighbors, neighborIndices, slotCount,
            neighborIdx)) {
        if (candidateCount < candidateCapacity) {
          candidateIndices[candidateCount] = neighborIdx;
          candidateDistSquared[candidateCount] = distSq;
          ++candidateCount;
        }
      }
    }

    const int toAddTarget = std::max(0, desiredSlots - activeCount);
    if (toAddTarget > 0 && candidateCount > 0) {
      const int limitedAdd = std::min(toAddTarget, candidateCount);
      for (int i = 0; i < limitedAdd; ++i) {
        const int neighborIdx = candidateIndices[i];
        if (containsNeighbor(activeNeighbors, neighborIndices, slotCount,
                             neighborIdx)) {
          continue;
        }
        const int slot = findFreeSlot(activeNeighbors, slotCount);
        if (slot < 0) {
          break;
        }
        maskSet(activeNeighbors, slot);
        neighborIndices[slot] = neighborIdx;
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = dt;
        }
        ++activeCount;
      }
    }

    if (selfParams.isPredator) {
      int &tgtIdx = buf.predatorTargetIndices[gIdx];
      float &tgtTime = buf.predatorTargetTimers[gIdx];
      tgtTime -= dt;

      std::array<float, kMaxPredatorCandidates> predatorDists{};
      const int predatorFetched = index.gatherNearest(
          pos, kMaxPredatorCandidates, maxPredatorAlertRadius,
          predatorCandidates.data(), predatorDists.data());

      for (int i = 0; i < predatorFetched; ++i) {
        const int preyIdx = predatorCandidates[i];
        if (preyIdx == gIdx || preyIdx < 0 || preyIdx >= totalCount) {
          continue;
        }
        const int preySid = buf.speciesIds[preyIdx];
        if (preySid < 0 || preySid >= static_cast<int>(globalSpeciesParams.size())) {
          continue;
        }
        if (globalSpeciesParams[preySid].isPredator) {
          continue;
        }

        const SpeciesParams &preyParams = globalSpeciesParams[preySid];
        float alertRadius = std::max(preyParams.predatorAlertRadius, 0.0f);
        if (alertRadius <= 0.0f) {
          alertRadius = maxPredatorAlertRadius;
        }
        const float alertRadiusSq = alertRadius * alertRadius;
        const float distSq = predatorDists[i];
        if (distSq >= alertRadiusSq || distSq < kEpsilon) {
          continue;
        }

        const float dist = glm::sqrt(std::max(distSq, kEpsilon));
        float escapeStrength =
            std::clamp(1.0f - dist / alertRadius, 0.0f, 1.0f);
        escapeStrength = escapeStrength * escapeStrength *
                         (3.0f - 2.0f * escapeStrength);

        const glm::vec3 diff = buf.positions[preyIdx] - pos;
        const glm::vec3 escapeDir = diff / dist;
        buf.predatorInfluences[preyIdx] += escapeDir * escapeStrength * 5.0f;
        buf.predatorThreats[preyIdx] =
            std::max(buf.predatorThreats[preyIdx], escapeStrength);
        buf.stresses[preyIdx] =
            std::max(buf.stresses[preyIdx],
                     std::clamp(0.4f + escapeStrength * 0.6f, 0.0f, 1.0f));

        if (predatorCandidateCount < kMaxPredatorCandidates) {
          predatorCandidates[predatorCandidateCount++] = preyIdx;
        }
      }

      const bool targetInvalid =
          (tgtIdx < 0) || (tgtTime <= 0.0f) ||
          buf.speciesIds[tgtIdx] < 0 ||
          buf.speciesIds[tgtIdx] >=
              static_cast<int>(globalSpeciesParams.size()) ||
          globalSpeciesParams[buf.speciesIds[tgtIdx]].isPredator;
      if (targetInvalid) {
        tgtIdx = -1;
        if (predatorCandidateCount > 0) {
          const int pick = predatorCandidates[nextRandom() % predatorCandidateCount];
          tgtIdx = pick;
          tgtTime = selfParams.tau;
        } else {
          tgtTime = 0.0f;
        }
      }

      if (tgtIdx >= 0 && tgtIdx < totalCount) {
        const glm::vec3 diff = buf.positions[tgtIdx] - pos;
        const float d2 = glm::dot(diff, diff);
        if (d2 > kEpsilon) {
          const glm::vec3 chaseDir = diff * (1.0f / glm::sqrt(d2));
          const glm::vec3 desiredVel = chaseDir * selfParams.maxSpeed;
          newAcceleration += desiredVel - vel;
        }
      }
    }

    const float phi =
        activeCount > 0
            ? static_cast<float>(activeCount) /
                  static_cast<float>(std::max(1, selfParams.maxNeighbors))
            : 0.0f;
    if (phi < 1.0f) {
      buf.isAttracting[gIdx] = 1;
      buf.attractTimers[gIdx] = selfParams.tau;
    } else if (buf.isAttracting[gIdx]) {
      buf.attractTimers[gIdx] -= dt;
      if (buf.attractTimers[gIdx] <= 0.0f) {
        buf.isAttracting[gIdx] = 0;
        buf.attractTimers[gIdx] = 0.0f;
      }
    }

    if (buf.isAttracting[gIdx]) {
      if (attractDirCount > 0) {
        const glm::vec3 avgDir = safeNormalize(
            attractDirSum / static_cast<float>(attractDirCount));
        const glm::vec3 desiredVel = avgDir * selfParams.maxSpeed;
        const glm::vec3 attractAcc =
            (desiredVel - vel) * selfParams.lambda;
        newAcceleration += attractAcc;
      }
    }

    if (activeCount == 0) {
      buf.accelerations[gIdx] = newAcceleration;
      continue;
    }

    glm::vec3 sumSep(0.0f);
    glm::vec3 sumAlign(0.0f);
    glm::vec3 sumCoh(0.0f);
    float stressGainSum = 0.0f;
    float stressWeightSum = 0.0f;

    const float selfStress = buf.stresses[gIdx];
    const float threatLevel = std::clamp(buf.predatorThreats[gIdx], 0.0f, 1.0f);
    const float propagationRadius = std::max(selfParams.cohesionRange, 1.0f);

    for (int slot = 0; slot < slotCount; ++slot) {
      if (!maskTest(activeNeighbors, slot)) {
        continue;
      }
      const int neighborIdx = neighborIndices[slot];
      if (neighborIdx < 0 || neighborIdx >= totalCount) {
        maskReset(activeNeighbors, slot);
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
        continue;
      }

      const int neighborSid = buf.speciesIds[neighborIdx];
      if (neighborSid < 0 ||
          neighborSid >= static_cast<int>(globalSpeciesParams.size())) {
        maskReset(activeNeighbors, slot);
        if (slot < static_cast<int>(cohesionMemories.size())) {
          cohesionMemories[slot] = 0.0f;
        }
        continue;
      }
      const SpeciesParams &neighborParams = globalSpeciesParams[neighborSid];

      const glm::vec3 diff = buf.positions[neighborIdx] - pos;
      const float distSq = glm::dot(diff, diff);
      if (distSq <= kEpsilon) {
        continue;
      }
      const float dist = glm::sqrt(distSq);

      const float selfRadius = std::max(selfParams.bodyRadius, 0.0f);
      const float neighborRadius = std::max(neighborParams.bodyRadius, 0.0f);
      const float combinedRadius = selfRadius + neighborRadius;
      if (combinedRadius > 1e-5f) {
        const float clearanceSq = combinedRadius * combinedRadius;
        if (distSq < clearanceSq) {
          const float distSafe = glm::sqrt(std::max(distSq, kEpsilon));
          const float penetration = clearanceSq > 0.0f
                                        ? std::max(combinedRadius - distSafe, 0.0f)
                                        : 0.0f;
          if (penetration > 0.0f) {
            const glm::vec3 normal = diff * (1.0f / distSafe);
            const float penetrationRatio =
                penetration / std::max(combinedRadius, 1e-5f);
            const float response = penetrationRatio * penetrationRatio * 10.0f;
            const float selfImpulse =
                response * std::max(selfParams.separation, 0.02f) *
                (1.0f + selfParams.maxSpeed);
            sumSep += normal * selfImpulse;
          }
        }
      }

      if (dist < propagationRadius) {
        const float neighborStress = buf.stresses[neighborIdx];
        if (neighborStress > selfStress) {
          const float stressStrength = smoothStep(0.25f, 0.75f, neighborStress);
          const float distanceFactor =
              std::clamp(1.0f - dist / propagationRadius, 0.0f, 1.0f);
          if (distanceFactor > 0.0f) {
            const float weight = stressStrength * distanceFactor;
            stressGainSum += neighborStress * weight;
            stressWeightSum += weight;
          }
        }
      }

      float separationRange = selfParams.separationRange;
      if (separationRange <= 1e-4f) {
        const float bodyDiameter = std::max(selfParams.bodyRadius * 2.0f, 0.0f);
        const float bodyLength = std::abs(selfParams.bodyHeadLength) +
                                 std::abs(selfParams.bodyTailLength);
        separationRange = std::max(bodyDiameter, bodyLength);
      }
      float wSep = 0.0f;
      if (separationRange > 1e-4f) {
        wSep = std::clamp(1.0f - (dist / separationRange), 0.0f, 1.0f);
      }
      sumSep += (-diff) * (wSep / std::max(distSq, kEpsilon));

      float wCoh = std::clamp(dist / std::max(selfParams.cohesionRange, 1e-4f),
                              0.0f, 1.0f);
      const float stressFactor = 1.0f + buf.stresses[gIdx] * 0.2f;
      const float cohesionThreatFactor =
          1.0f + gSimulationTuning.cohesionBoost * threatLevel;
      wCoh *= stressFactor * cohesionThreatFactor;
      sumCoh += buf.positions[neighborIdx] * wCoh;

      sumAlign += buf.velocities[neighborIdx];
    }

    if (stressWeightSum > 0.0f) {
      const float propagatedStress = stressGainSum / stressWeightSum;
      const float delta = propagatedStress - selfStress;
      if (delta > 0.0f) {
        const float blend = std::clamp(gSimulationTuning.threatGain * dt, 0.0f,
                                       0.6f);
        buf.stresses[gIdx] =
            std::clamp(selfStress + delta * blend, 0.0f, 1.0f);
      }
    }

    glm::vec3 totalSeparation(0.0f);
    const float sepLen2 = glm::length2(sumSep);
    if (sepLen2 > kMinDirLen2) {
      const float separationThreatFactor =
          std::lerp(1.0f, gSimulationTuning.separationMinFactor, threatLevel);
      totalSeparation = safeNormalize(sumSep) *
                        (selfParams.separation * separationThreatFactor);
    }

    glm::vec3 totalCohesion(0.0f);
    if (activeCount > 0) {
      const glm::vec3 avgCohPos =
          sumCoh * (1.0f / static_cast<float>(activeCount));
      glm::vec3 cohDir = avgCohPos - pos;
      const glm::vec3 cohNorm = safeNormalize(cohDir);
      if (glm::length2(cohNorm) > kMinDirLen2) {
        totalCohesion = cohNorm * selfParams.cohesion;
      }
    }

    glm::vec3 totalAlignment(0.0f);
    glm::vec3 torqueImpulse(0.0f);
    if (activeCount > 0) {
      glm::vec3 avgAlignVel =
          sumAlign * (1.0f / static_cast<float>(activeCount));
      glm::vec3 aliDir = avgAlignVel - vel;
      const float aliLen2 = glm::length2(aliDir);
      if (aliLen2 > kMinDirLen2) {
        const float alignmentThreatFactor =
            1.0f + gSimulationTuning.alignmentBoost * threatLevel;
        totalAlignment = safeNormalize(aliDir) *
                          (selfParams.alignment * alignmentThreatFactor);
      }
    }

    if (velLen2 > kMinDirLen2) {
      const float aliLen2 = glm::length2(totalAlignment);
      if (aliLen2 > kMinDirLen2) {
        const float velLen = glm::sqrt(velLen2);
        const glm::vec3 forward2 = vel * (1.0f / velLen);
        glm::vec3 tgt2 = totalAlignment * (1.0f / glm::sqrt(aliLen2));
        float dot2 = std::clamp(glm::dot(forward2, tgt2), -1.0f, 1.0f);
        float ang2 = std::acos(dot2);
        if (ang2 > 1e-4f) {
          glm::vec3 axis2 = glm::cross(forward2, tgt2);
          const float axisLen2 = glm::length2(axis2);
          if (axisLen2 > 1e-8f) {
            axis2 *= (1.0f / glm::sqrt(axisLen2));
            const float rot2 =
                std::min(ang2, selfParams.torqueStrength * dt);
            const float rotRatio = rot2 / std::max(ang2, 1e-4f);
            totalAlignment *= rotRatio;
            torqueImpulse += axis2 * (ang2 * selfParams.torqueStrength);
          }
        }
      }
    }

    newAcceleration += totalSeparation + totalAlignment + totalCohesion +
                       torqueImpulse;

    buf.accelerations[gIdx] = newAcceleration;
  }
}

void updateBoidKinematicsRange(SoABuffers &buf, int begin, int end, float dt) {
  const int totalCount = static_cast<int>(buf.positions.size());
  if (totalCount == 0) {
    return;
  }
  begin = std::clamp(begin, 0, totalCount);
  end = std::clamp(end, begin, totalCount);
  if (begin >= end) {
    return;
  }

  for (int gIdx = begin; gIdx < end; ++gIdx) {
    const int sid = buf.speciesIds[gIdx];
    if (sid < 0 || sid >= static_cast<int>(globalSpeciesParams.size())) {
      continue;
    }
    const SpeciesParams &params = globalSpeciesParams[sid];

    const glm::vec3 velocity = buf.velocities[gIdx];
    const glm::vec3 acceleration = buf.accelerations[gIdx];
    const glm::vec3 position = buf.positions[gIdx];

    float currentStress = buf.stresses[gIdx];
    float stressFactor = 1.0f;
    if (currentStress > 0.95f) {
      const float t = (currentStress - 0.8f) / 0.2f;
      stressFactor = 1.0f + 1.6f + t * 0.9f;
    } else if (currentStress > 0.7f) {
      const float t = (currentStress - 0.2f) / 0.6f;
      const float smoothT = t * t * (3.0f - 2.0f * t);
      stressFactor = 1.0f + 0.4f + smoothT * 1.8f;
    } else {
      const float t = currentStress / 0.2f;
      stressFactor = 1.0f + t * 0.4f;
    }

    glm::vec3 desiredVelocity = velocity + acceleration * dt;
    float speed = 0.0f;
    glm::vec3 oldDir = velocity;
    float oldSpeedSq = glm::length2(velocity);
    if (oldSpeedSq > kMinDirLen2) {
      oldDir /= glm::sqrt(oldSpeedSq);
    } else {
      oldDir = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    const float desiredSpeedSq = glm::length2(desiredVelocity);
    glm::vec3 newDir = oldDir;
    if (desiredSpeedSq > kMinDirLen2) {
      speed = glm::sqrt(desiredSpeedSq);
      newDir = desiredVelocity / speed;
    }

    float maxTurnAngle = params.maxTurnAngle * stressFactor;
    if (params.isPredator && buf.predatorTargetIndices[gIdx] >= 0) {
      maxTurnAngle *= 1.5f;
    }
    if (currentStress > 0.7f) {
      const float emergencyTurnFactor = 1.0f + (currentStress - 0.7f) * 5.0f;
      maxTurnAngle *= emergencyTurnFactor;
    }

    const float dotProduct = glm::dot(oldDir, newDir);
    float angle = std::acos(std::clamp(dotProduct, -1.0f, 1.0f));
    if (angle > maxTurnAngle) {
      glm::vec3 axis = glm::cross(oldDir, newDir);
      const float axisLen2 = glm::length2(axis);
      if (axisLen2 > 1e-8f) {
        axis /= glm::sqrt(axisLen2);
        const float rot = std::min(angle, maxTurnAngle * dt);
        newDir = approxRotate(oldDir, axis, rot);
      }
    }

    const float tilt = newDir.y;
    if (std::fabs(tilt) > 1e-4f) {
      glm::vec3 flatDir = glm::normalize(glm::vec3(newDir.x, 0.0f, newDir.z));
      glm::vec3 axis = glm::cross(newDir, flatDir);
      const float flatAngle =
          std::acos(std::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f));
      const float axisLen2 = glm::length2(axis);
      if (flatAngle > 1e-4f && axisLen2 > 1e-8f) {
        axis /= glm::sqrt(axisLen2);
        const float rot = std::min(flatAngle, params.horizontalTorque * dt);
        newDir = approxRotate(newDir, axis, rot);
      }
    }

    const float maxSpeed = params.maxSpeed * stressFactor;
    const float finalSpeed = std::clamp(speed, params.minSpeed, maxSpeed);
    const glm::vec3 newVelocity = newDir * finalSpeed;

    buf.velocitiesWrite[gIdx] = newVelocity;
    buf.positionsWrite[gIdx] = position + newVelocity * dt;
    buf.orientationsWrite[gIdx] = dirToQuatRollZero(newDir);
    buf.accelerations[gIdx] = glm::vec3(0.0f);

    buf.predatorInfluences[gIdx] *= 0.5f;
    buf.predatorThreats[gIdx] =
        std::max(buf.predatorThreats[gIdx] - gSimulationTuning.threatDecay * dt,
                 0.0f);

    if (buf.stresses[gIdx] > 0.0f) {
      const float decayRate = 1.0f;
      buf.stresses[gIdx] -= easeOut(dt * decayRate);
      if (buf.stresses[gIdx] < 0.0f) {
        buf.stresses[gIdx] = 0.0f;
      }
    }
  }
}
