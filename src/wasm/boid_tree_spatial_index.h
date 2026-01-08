#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "boid_unit.h"
#include "spatial_index.h"

// BoidUnitツリーを使う SpatialIndex 実装。
// BoidSimulation(=世界状態)から空間クエリ実装を分離し、差し替え可能にする。
class BoidTreeSpatialIndex final : public SpatialIndex {
public:
  BoidTreeSpatialIndex() = default;
  explicit BoidTreeSpatialIndex(const BoidUnit *root) : root_(root) {}

  void setRoot(const BoidUnit *root) { root_ = root; }
  const BoidUnit *getRoot() const { return root_; }

  // SpatialIndex implementation
  void forEachLeaf(const LeafVisitor &visitor) const override;
  void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                     const LeafVisitor &visitor) const override;

  // 球交差クエリ（早期終了対応）。visitor が false を返すと探索を打ち切る。
  // NOTE: 反復DFS + thread_local スタック再利用で、ホットパスの割り当てを避ける。
  // NOTE: thread_local を使うため同一スレッド内で再入不可（ネスト呼び出し禁止）。
  template <typename CancelableVisitor>
  void forEachLeafIntersectingSphereCancelable(const glm::vec3 &center,
                                               float radius,
                                               CancelableVisitor &&visitor) const {
    if (!root_) {
      return;
    }

    // NOTE: フレーム内で多回呼ばれるため、ローカルvectorの生成・破棄を避ける。
    static thread_local std::vector<std::pair<const BoidUnit *, const BoidUnit *>> stack;
    stack.clear();
    if (stack.capacity() < 256) {
      stack.reserve(256);
    }
    stack.emplace_back(root_, nullptr);

    // NOTE: ツリー破損で循環が混入した場合の安全弁（到達すると早期return）。
    constexpr std::size_t kMaxTraversalSteps = 5'000'000;
    std::size_t steps = 0;

    while (!stack.empty()) {
      const auto currentPair = stack.back();
      stack.pop_back();
      const BoidUnit *current = currentPair.first;
      const BoidUnit *parent = currentPair.second;

      if (!current) {
        continue;
      }
      if (++steps > kMaxTraversalSteps) {
        return;
      }

      const glm::vec3 delta = current->center - center;
      const float maxDist = current->radius + radius;
      if (glm::dot(delta, delta) > maxDist * maxDist) {
        continue;
      }

      if (current->children.empty()) {
        SpatialLeaf leaf{current->indices.data(), current->indices.size(), current};
        if (!visitor(leaf)) {
          return;
        }
        continue;
      }

      for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
        const BoidUnit *child = *it;
        if (!child) {
          continue;
        }
        if (child == current || child == parent) {
          continue;
        }
        stack.emplace_back(child, current);
      }
    }
  }

private:
  void forEachLeafRecursive(const BoidUnit *node, const LeafVisitor &visitor) const;
  void forEachLeafIntersectingSphereRecursive(const BoidUnit *node,
                                              const glm::vec3 &center,
                                              float radius,
                                              const LeafVisitor &visitor) const;

  const BoidUnit *root_ = nullptr;
};
