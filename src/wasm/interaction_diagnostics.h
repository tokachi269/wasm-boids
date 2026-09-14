#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

struct InteractionDiagnostics {
  uint64_t boidsProcessed = 0;
  uint64_t schoolCandidatesVisited = 0;
  uint64_t schoolCandidatesEligible = 0;
  uint64_t schoolAssociations = 0;
  uint64_t schoolInfluencePositive = 0;
  uint64_t cachedEntriesValidated = 0;
  uint64_t cachedEntriesRemoved = 0;
  uint64_t leafCandidatesChecked = 0;
  uint64_t leafCandidatesAlreadyCached = 0;
  uint64_t leafCandidatesInRange = 0;
  uint64_t leafCandidatesInFov = 0;
  uint64_t leafCandidateSearchBoids = 0;
  uint64_t leafCandidateSearchStartingActiveSum = 0;
  uint64_t leafCandidateSearchLeafMembersSum = 0;
  uint64_t cacheInserts = 0;
  uint64_t externalQueries = 0;
  uint64_t externalCandidatesVisited = 0;
  uint64_t externalCandidatesAccepted = 0;
  uint64_t cachedNeighborsAggregated = 0;
  uint64_t externalNeighborsAggregated = 0;
  uint64_t penetrationTests = 0;
  uint64_t penetrations = 0;
  uint64_t forwardNormalizations = 0;
  uint64_t schoolDistanceSquareRoots = 0;
  uint64_t neighborDistanceSquareRoots = 0;
  uint64_t forceNormalizations = 0;
  std::array<uint64_t, 33> neighborCountHistogram{};
  std::array<uint64_t, 33> candidateSearchStartingActiveHistogram{};
  std::array<uint64_t, 5> penetrationRatioHistogram{};
  double penetrationRatioSum = 0.0;
  double penetrationImpulseSum = 0.0;
  float penetrationRatioMax = 0.0f;
  float penetrationImpulseMax = 0.0f;

  void recordPenetration(float ratio, float impulse) {
    const std::size_t bucket = ratio < 0.05f ? 0
        : ratio < 0.10f ? 1
        : ratio < 0.25f ? 2
        : ratio < 0.50f ? 3
        : 4;
    ++penetrationRatioHistogram[bucket];
    penetrationRatioSum += ratio;
    penetrationImpulseSum += impulse;
    if (ratio > penetrationRatioMax) penetrationRatioMax = ratio;
    if (impulse > penetrationImpulseMax) penetrationImpulseMax = impulse;
  }

  void merge(const InteractionDiagnostics &other) {
    boidsProcessed += other.boidsProcessed;
    schoolCandidatesVisited += other.schoolCandidatesVisited;
    schoolCandidatesEligible += other.schoolCandidatesEligible;
    schoolAssociations += other.schoolAssociations;
    schoolInfluencePositive += other.schoolInfluencePositive;
    cachedEntriesValidated += other.cachedEntriesValidated;
    cachedEntriesRemoved += other.cachedEntriesRemoved;
    leafCandidatesChecked += other.leafCandidatesChecked;
    leafCandidatesAlreadyCached += other.leafCandidatesAlreadyCached;
    leafCandidatesInRange += other.leafCandidatesInRange;
    leafCandidatesInFov += other.leafCandidatesInFov;
    leafCandidateSearchBoids += other.leafCandidateSearchBoids;
    leafCandidateSearchStartingActiveSum +=
        other.leafCandidateSearchStartingActiveSum;
    leafCandidateSearchLeafMembersSum += other.leafCandidateSearchLeafMembersSum;
    cacheInserts += other.cacheInserts;
    externalQueries += other.externalQueries;
    externalCandidatesVisited += other.externalCandidatesVisited;
    externalCandidatesAccepted += other.externalCandidatesAccepted;
    cachedNeighborsAggregated += other.cachedNeighborsAggregated;
    externalNeighborsAggregated += other.externalNeighborsAggregated;
    penetrationTests += other.penetrationTests;
    penetrations += other.penetrations;
    forwardNormalizations += other.forwardNormalizations;
    schoolDistanceSquareRoots += other.schoolDistanceSquareRoots;
    neighborDistanceSquareRoots += other.neighborDistanceSquareRoots;
    forceNormalizations += other.forceNormalizations;
    penetrationRatioSum += other.penetrationRatioSum;
    penetrationImpulseSum += other.penetrationImpulseSum;
    if (other.penetrationRatioMax > penetrationRatioMax) {
      penetrationRatioMax = other.penetrationRatioMax;
    }
    if (other.penetrationImpulseMax > penetrationImpulseMax) {
      penetrationImpulseMax = other.penetrationImpulseMax;
    }
    for (std::size_t i = 0; i < neighborCountHistogram.size(); ++i) {
      neighborCountHistogram[i] += other.neighborCountHistogram[i];
      candidateSearchStartingActiveHistogram[i] +=
          other.candidateSearchStartingActiveHistogram[i];
    }
    for (std::size_t i = 0; i < penetrationRatioHistogram.size(); ++i) {
      penetrationRatioHistogram[i] += other.penetrationRatioHistogram[i];
    }
  }
};
