#pragma once

#include "spatial_index.h"
#include <glm/vec3.hpp>
#include <utility>

// SpatialIndex を用いた球領域クエリのヘルパー郡
namespace spatial_query {

// 任意のコールバックを使って球内の Boid index を列挙する簡易ユーティリティ
// これにより呼び出し側が葉ノード走査ループを重複して書かずに済む。

template <typename Visitor>
inline void forEachBoidInSphere(const SpatialIndex &index,
                                const glm::vec3 &center, float radius,
                                Visitor &&visitor) {
  index.forEachLeafIntersectingSphere(
      center, radius, [&](const SpatialLeaf &leaf) {
        for (std::size_t i = 0; i < leaf.count; ++i) {
          visitor(leaf.indices[i], leaf.node);
        }
      });
}

namespace detail {

// BoidSimulation など cancelable API を持つ型ならそちらを使う（SFINAE）。
template <typename Index, typename CancelableVisitor>
inline auto forEachLeafIntersectingSphereCancelable(
    const Index &index, const glm::vec3 &center, float radius,
    CancelableVisitor &&visitor, int)
    -> decltype(index.forEachLeafIntersectingSphereCancelable(
                    center, radius, std::forward<CancelableVisitor>(visitor)),
                void()) {
  index.forEachLeafIntersectingSphereCancelable(
      center, radius, std::forward<CancelableVisitor>(visitor));
}

// フォールバック: cancelできない場合は最後まで走査する（limit は leaf 内ループでのみ効く）。
template <typename Index, typename CancelableVisitor>
inline void forEachLeafIntersectingSphereCancelable(
    const Index &index, const glm::vec3 &center, float radius,
    CancelableVisitor &&visitor, long) {
  index.forEachLeafIntersectingSphere(center, radius,
                                      [&](const SpatialLeaf &leaf) {
                                        (void)visitor(leaf);
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
  detail::forEachLeafIntersectingSphereCancelable(
      index, center, radius,
      [&](const SpatialLeaf &leaf) -> bool {
        for (std::size_t i = 0; i < leaf.count; ++i) {
          visitor(leaf.indices[i], leaf.node);
          if (--remaining == 0) {
            return false;
          }
        }
        return true;
      },
      0);
}

} // namespace spatial_query
