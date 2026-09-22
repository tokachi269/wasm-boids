#pragma once
#include <cstddef>
#include <functional>
#include <glm/vec3.hpp>

// 空間インデックスが管理する局所グループの読み取り専用summary。
// treeのleafやBoidUnitを公開せず、cluster集計とstorage reorderに必要な値だけを渡す。
struct SpatialGroup {
  const int *indices;
  std::size_t count;
  int id;
  int speciesId;
  glm::vec3 center;
  glm::vec3 averageVelocity;
  float radius;
};

class SpatialIndex {
public:
  using GroupVisitor = std::function<void(const SpatialGroup &)>;
  using CandidateVisitor = std::function<void(int boidIndex, int groupId)>;
  virtual ~SpatialIndex() = default;

  virtual void forEachGroup(const GroupVisitor &visitor) const = 0;

  virtual void forEachCandidateIntersectingSphere(
      const glm::vec3 &center, float radius,
      const CandidateVisitor &visitor) const = 0;
};
