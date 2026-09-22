#pragma once

#include "spatial_index.h"
#include <glm/vec3.hpp>
#include <utility>

// SpatialIndex を用いた球領域クエリのヘルパー群
namespace spatial_query {

// 任意のコールバックを使って球内の Boid index を列挙する簡易ユーティリティ
// 呼び出し側はtreeのnodeやleafを参照しない。

template <typename Visitor>
inline void forEachBoidInSphere(const SpatialIndex &index,
                                const glm::vec3 &center, float radius,
                                Visitor &&visitor) {
  index.forEachCandidateIntersectingSphere(
      center, radius, std::forward<Visitor>(visitor));
}

namespace detail {

// BoidSimulation など cancelable API を持つ型ならそちらを使う（SFINAE）。
template <typename Index, typename CancelableVisitor>
inline auto forEachCandidateIntersectingSphereCancelable(
    const Index &index, const glm::vec3 &center, float radius,
    CancelableVisitor &&visitor, int)
    -> decltype(index.forEachCandidateIntersectingSphereCancelable(
                    center, radius, std::forward<CancelableVisitor>(visitor)),
                void()) {
  index.forEachCandidateIntersectingSphereCancelable(
      center, radius, std::forward<CancelableVisitor>(visitor));
}

// フォールバック: underlying indexは最後まで走査するが、visitor呼び出しは打ち切る。
template <typename Index, typename CancelableVisitor>
inline void forEachCandidateIntersectingSphereCancelable(
    const Index &index, const glm::vec3 &center, float radius,
    CancelableVisitor &&visitor, long) {
  bool active = true;
  index.forEachCandidateIntersectingSphere(
      center, radius, [&](int boidIndex, int groupId) {
        if (active) {
          active = visitor(boidIndex, groupId);
        }
      });
}

} // namespace detail

// 球内の Boid を列挙する（上限付き、必要数に達したら探索を打ち切る）。
// - 近傍補完や捕食者ターゲット候補など「全部は要らない」用途で空間探索コストを抑える。
template <typename Index, typename Visitor>
inline void forEachBoidInSphereLimited(const Index &index,
                                       const glm::vec3 &center, float radius,
                                       std::size_t limit, Visitor &&visitor) {
  if (limit == 0) {
    return;
  }

  std::size_t remaining = limit;
  detail::forEachCandidateIntersectingSphereCancelable(
      index, center, radius,
      [&](int boidIndex, int groupId) -> bool {
        visitor(boidIndex, groupId);
        if (--remaining == 0) {
          return false;
        }
        return true;
      },
      0);
}

} // namespace spatial_query
