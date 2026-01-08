#include "boid_tree_spatial_index.h"

#include <cstddef>

#include <glm/gtx/norm.hpp>

// 非ホットパス向けの素直な実装。ホットパスは cancelable 版（反復DFS）を使う。
void BoidTreeSpatialIndex::forEachLeafRecursive(const BoidUnit *node,
                                               const LeafVisitor &visitor) const {
  if (!node) {
    return;
  }

  if (node->children.empty()) {
    SpatialLeaf leaf{node->indices.data(), node->indices.size(), node};
    visitor(leaf);
    return;
  }

  for (const auto *child : node->children) {
    if (!child) {
      continue;
    }
    forEachLeafRecursive(child, visitor);
  }
}

void BoidTreeSpatialIndex::forEachLeaf(const LeafVisitor &visitor) const {
  if (!root_) {
    return;
  }
  forEachLeafRecursive(root_, visitor);
}

void BoidTreeSpatialIndex::forEachLeafIntersectingSphereRecursive(
    const BoidUnit *node, const glm::vec3 &center, float radius,
    const LeafVisitor &visitor) const {
  if (!node) {
    return;
  }

  const glm::vec3 delta = node->center - center;
  const float maxDist = node->radius + radius;

  if (glm::dot(delta, delta) > maxDist * maxDist) {
    return;
  }

  if (node->children.empty()) {
    SpatialLeaf leaf{node->indices.data(), node->indices.size(), node};
    visitor(leaf);
    return;
  }

  for (const auto *child : node->children) {
    if (!child) {
      continue;
    }
    forEachLeafIntersectingSphereRecursive(child, center, radius, visitor);
  }
}

void BoidTreeSpatialIndex::forEachLeafIntersectingSphere(
    const glm::vec3 &center, float radius, const LeafVisitor &visitor) const {
  if (!root_) {
    return;
  }
  forEachLeafIntersectingSphereRecursive(root_, center, radius, visitor);
}
