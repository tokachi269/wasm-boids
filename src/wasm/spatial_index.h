#pragma once
#include <cstddef>
#include <functional>
#include <glm/vec3.hpp>

class BoidUnit;

struct SpatialLeaf {
  const int *indices; // pointer to contiguous boid indices
  std::size_t count;  // number of indices in this leaf
  const BoidUnit *node; // source node for optional metadata access
};

class SpatialIndex {
public:
  using LeafVisitor = std::function<void(const SpatialLeaf &)>;
  virtual ~SpatialIndex() = default;

  virtual void forEachLeaf(const LeafVisitor &visitor) const = 0;

  virtual void forEachLeafIntersectingSphere(const glm::vec3 &center,
                                             float radius,
                                             const LeafVisitor &visitor) const = 0;
};
