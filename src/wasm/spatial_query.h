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

} // namespace spatial_query
