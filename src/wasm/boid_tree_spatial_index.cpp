#include "boid_tree_spatial_index.h"

#include <cstddef>

#include <glm/gtx/norm.hpp>

// 非ホットパス向けの素直な実装。ホットパスは cancelable 版（反復DFS）を使う。
void BoidTreeSpatialIndex::forEachGroupRecursive(
    const BoidUnit *node, const GroupVisitor &visitor) const {
  if (!node) {
    return;
  }

  if (node->children.empty()) {
    visitor(SpatialGroup{node->indices.data(), node->indices.size(), node->id,
                         node->speciesId, node->center, node->averageVelocity,
                         node->radius});
    return;
  }

  for (const auto *child : node->children) {
    if (!child) {
      continue;
    }
    forEachGroupRecursive(child, visitor);
  }
}

void BoidTreeSpatialIndex::forEachGroup(const GroupVisitor &visitor) const {
  if (!root_) {
    return;
  }
  forEachGroupRecursive(root_, visitor);
}

void BoidTreeSpatialIndex::ensureGroupMembership(std::size_t boidCount) const {
  if (groupMembershipValid_ && groupByBoid_.size() == boidCount) {
    return;
  }
  groupByBoid_.assign(boidCount, nullptr);
  rebuildGroupMembershipRecursive(root_);
  groupMembershipValid_ = true;
}

void BoidTreeSpatialIndex::invalidateGroupMembership() const {
  groupMembershipValid_ = false;
}

void BoidTreeSpatialIndex::rebuildGroupMembershipRecursive(
    const BoidUnit *node) const {
  if (!node) {
    return;
  }
  if (!node->children.empty()) {
    for (const BoidUnit *child : node->children) {
      rebuildGroupMembershipRecursive(child);
    }
    return;
  }
  for (int boidIndex : node->indices) {
    if (boidIndex >= 0 &&
        static_cast<std::size_t>(boidIndex) < groupByBoid_.size()) {
      groupByBoid_[static_cast<std::size_t>(boidIndex)] = node;
    }
  }
}

bool BoidTreeSpatialIndex::localGroupForBoid(int boidIndex,
                                             SpatialGroup &group) const {
  if (!groupMembershipValid_ || boidIndex < 0 ||
      static_cast<std::size_t>(boidIndex) >= groupByBoid_.size()) {
    return false;
  }
  const BoidUnit *node = groupByBoid_[static_cast<std::size_t>(boidIndex)];
  if (!node) {
    return false;
  }
  group = SpatialGroup{node->indices.data(), node->indices.size(), node->id,
                       node->speciesId, node->center, node->averageVelocity,
                       node->radius};
  return true;
}

void BoidTreeSpatialIndex::forEachCandidateIntersectingSphereRecursive(
    const BoidUnit *node, const glm::vec3 &center, float radius,
    const CandidateVisitor &visitor) const {
  if (!node) {
    return;
  }

  const glm::vec3 delta = node->center - center;
  const float maxDist = node->radius + radius;

  if (glm::dot(delta, delta) > maxDist * maxDist) {
    return;
  }

  if (node->children.empty()) {
    for (int index : node->indices) {
      visitor(index, node->id);
    }
    return;
  }

  for (const auto *child : node->children) {
    if (!child) {
      continue;
    }
    forEachCandidateIntersectingSphereRecursive(child, center, radius, visitor);
  }
}

void BoidTreeSpatialIndex::forEachCandidateIntersectingSphere(
    const glm::vec3 &center, float radius,
    const CandidateVisitor &visitor) const {
  if (!root_) {
    return;
  }
  forEachCandidateIntersectingSphereRecursive(root_, center, radius, visitor);
}
