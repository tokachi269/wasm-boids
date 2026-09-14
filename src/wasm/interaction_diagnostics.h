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
    for (std::size_t i = 0; i < neighborCountHistogram.size(); ++i) {
      neighborCountHistogram[i] += other.neighborCountHistogram[i];
    }
  }
};
