#include "lbvh_index.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

inline uint32_t expandBits(uint32_t v) {
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

inline uint32_t morton3D(uint32_t x, uint32_t y, uint32_t z) {
  return (expandBits(x) << 2) | (expandBits(y) << 1) | expandBits(z);
}

struct Primitive {
  uint32_t morton = 0;
  int index = 0;
};

} // namespace

LbvhIndex::LbvhIndex(int maxLeafSize) : maxLeafSize_(std::max(1, maxLeafSize)) {}

void LbvhIndex::build(const SoABuffers &buffers) {
  buffers_ = &buffers;

  const std::size_t count = buffers.positions.size();
  nodes_.clear();
  leafRecords_.clear();
  leafIndexStorage_.clear();
  mortonOrder_.clear();
  boidToLeafIndex_.clear();

  if (count == 0) {
    return;
  }

  boidToLeafIndex_.assign(count, -1);

  glm::vec3 boundsMin(std::numeric_limits<float>::max());
  glm::vec3 boundsMax(-std::numeric_limits<float>::max());
  for (const auto &pos : buffers.positions) {
    boundsMin = vecMin(boundsMin, pos);
    boundsMax = vecMax(boundsMax, pos);
  }

  glm::vec3 extent = boundsMax - boundsMin;
  extent.x = extent.x <= 1e-6f ? 1.0f : extent.x;
  extent.y = extent.y <= 1e-6f ? 1.0f : extent.y;
  extent.z = extent.z <= 1e-6f ? 1.0f : extent.z;

  std::vector<Primitive> primitives;
  primitives.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const glm::vec3 normalized = (buffers.positions[i] - boundsMin) / extent;
    const glm::vec3 clamped = glm::clamp(normalized, glm::vec3(0.0f),
                                         glm::vec3(0.999999f));
    constexpr float kScale = 1024.0f; // 10bits per axis
    const uint32_t ix = static_cast<uint32_t>(clamped.x * kScale);
    const uint32_t iy = static_cast<uint32_t>(clamped.y * kScale);
    const uint32_t iz = static_cast<uint32_t>(clamped.z * kScale);
    primitives.push_back(Primitive{morton3D(ix, iy, iz), static_cast<int>(i)});
  }

  std::sort(primitives.begin(), primitives.end(),
            [](const Primitive &a, const Primitive &b) {
              if (a.morton == b.morton)
                return a.index < b.index;
              return a.morton < b.morton;
            });

  mortonOrder_.reserve(count);
  for (const auto &prim : primitives) {
    mortonOrder_.push_back(prim.index);
  }

  nodes_.reserve(static_cast<std::size_t>(2 * count));
  buildRecursive(0, static_cast<int>(count));
}

void LbvhIndex::forEachLeaf(const LeafVisitor &visitor) const {
  if (!visitor)
    return;
  for (const auto &leaf : leafRecords_) {
    const int *ptr = leafIndexStorage_.data() + leaf.offset;
    visitor(SpatialLeaf{ptr, static_cast<std::size_t>(leaf.count), nullptr});
  }
}

void LbvhIndex::forEachLeafIntersectingSphere(
    const glm::vec3 &center, float radius, const LeafVisitor &visitor) const {
  if (!visitor || nodes_.empty())
    return;

  const float radiusSq = radius * radius;
  std::vector<int> stack;
  stack.reserve(64);
  stack.push_back(0); // root index

  while (!stack.empty()) {
    const int nodeIndex = stack.back();
    stack.pop_back();
    const Node &node = nodes_[nodeIndex];

    const glm::vec3 clamped = glm::clamp(center, node.boundsMin, node.boundsMax);
    const glm::vec3 diff = clamped - center;
    const float distSq = glm::dot(diff, diff);
    if (distSq > radiusSq)
      continue;

    if (node.isLeaf) {
      const int *ptr = leafIndexStorage_.data() + node.begin;
      visitor(SpatialLeaf{ptr,
                          static_cast<std::size_t>(node.end - node.begin),
                          nullptr});
    } else {
      if (node.left >= 0)
        stack.push_back(node.left);
      if (node.right >= 0)
        stack.push_back(node.right);
    }
  }
}

namespace {

float distanceToAabbSq(const glm::vec3 &point, const glm::vec3 &minBounds,
                       const glm::vec3 &maxBounds) {
  const glm::vec3 clamped = glm::clamp(point, minBounds, maxBounds);
  const glm::vec3 diff = clamped - point;
  return glm::dot(diff, diff);
}

} // namespace

int LbvhIndex::gatherNearest(const glm::vec3 &center, int maxCount,
                             float maxRadius, int *outIndices,
                             float *outDistancesSq) const {
  if (maxCount <= 0 || !outIndices || !outDistancesSq || nodes_.empty() ||
      !buffers_ || buffers_->positions.empty()) {
    return 0;
  }

  stats_.queries.fetch_add(1, std::memory_order_relaxed);

  int localNodesVisited = 0;
  int localLeavesVisited = 0;
  int localBoidsConsidered = 0;
  int localMaxQueue = 0;

  const float maxRadiusSq = (maxRadius > 0.0f)
                                ? (maxRadius * maxRadius)
                                : std::numeric_limits<float>::infinity();

  struct QueueCompare {
    bool operator()(const QueueEntry &a, const QueueEntry &b) const {
      return a.distSq > b.distSq;
    }
  };

  auto &queue = queryQueue_;
  queue.clear();
  queue.reserve(nodes_.size());

  QueueCompare compare;
  auto pushNode = [&](int nodeIndex, float distSq) {
    queue.push_back(QueueEntry{nodeIndex, distSq});
    std::push_heap(queue.begin(), queue.end(), compare);
    if (static_cast<int>(queue.size()) > localMaxQueue) {
      localMaxQueue = static_cast<int>(queue.size());
    }
  };

  auto popNode = [&]() {
    std::pop_heap(queue.begin(), queue.end(), compare);
    QueueEntry entry = queue.back();
    queue.pop_back();
    return entry;
  };

  pushNode(0, distanceToAabbSq(center, nodes_[0].boundsMin,
                               nodes_[0].boundsMax));

  auto &best = neighborScratch_;
  best.clear();
  best.reserve(static_cast<std::size_t>(maxCount));
  float currentWorstSq = maxRadiusSq;

  auto updateWorst = [&]() {
    if (static_cast<int>(best.size()) < maxCount) {
      currentWorstSq = maxRadiusSq;
      return;
    }
    float worst = best.front().first;
    for (const auto &entry : best) {
      if (entry.first > worst) {
        worst = entry.first;
      }
    }
    currentWorstSq = std::min(maxRadiusSq, worst);
  };

  while (!queue.empty()) {
    if (!best.empty() && static_cast<int>(best.size()) >= maxCount &&
        queue.front().distSq > currentWorstSq) {
      break;
    }

    const QueueEntry entry = popNode();
    ++localNodesVisited;

    if (entry.distSq > currentWorstSq) {
      continue;
    }

    const Node &node = nodes_[entry.nodeIndex];
    if (node.isLeaf) {
      ++localLeavesVisited;
      for (int i = node.begin; i < node.end; ++i) {
        const int boidIdx = leafIndexStorage_[i];
        ++localBoidsConsidered;
        const glm::vec3 &pos = buffers_->positions[boidIdx];
        const glm::vec3 diff = pos - center;
        const float distSq = glm::dot(diff, diff);
        if (distSq > currentWorstSq || distSq > maxRadiusSq) {
          continue;
        }

        if (static_cast<int>(best.size()) < maxCount) {
          best.emplace_back(distSq, boidIdx);
          updateWorst();
        } else {
          auto worstIt = std::max_element(
              best.begin(), best.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
          if (worstIt != best.end() && distSq < worstIt->first) {
            *worstIt = std::make_pair(distSq, boidIdx);
            updateWorst();
          }
        }
      }
    } else {
      if (node.left >= 0) {
        const float childDist = distanceToAabbSq(center,
                                                 nodes_[node.left].boundsMin,
                                                 nodes_[node.left].boundsMax);
        if (childDist <= currentWorstSq) {
          pushNode(node.left, childDist);
        }
      }
      if (node.right >= 0) {
        const float childDist = distanceToAabbSq(center,
                                                 nodes_[node.right].boundsMin,
                                                 nodes_[node.right].boundsMax);
        if (childDist <= currentWorstSq) {
          pushNode(node.right, childDist);
        }
      }
    }
  }

  stats_.nodesVisited.fetch_add(localNodesVisited, std::memory_order_relaxed);
  stats_.leavesVisited.fetch_add(localLeavesVisited,
                                 std::memory_order_relaxed);
  stats_.boidsConsidered.fetch_add(localBoidsConsidered,
                                   std::memory_order_relaxed);
  int prevMax = stats_.maxQueueSize.load(std::memory_order_relaxed);
  while (localMaxQueue > prevMax &&
         !stats_.maxQueueSize.compare_exchange_weak(
             prevMax, localMaxQueue, std::memory_order_relaxed)) {
  }

  std::sort(best.begin(), best.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  const int count = std::min(maxCount, static_cast<int>(best.size()));
  for (int i = 0; i < count; ++i) {
    outDistancesSq[i] = best[static_cast<std::size_t>(i)].first;
    outIndices[i] = best[static_cast<std::size_t>(i)].second;
  }
  return count;
}

bool LbvhIndex::getLeafMembers(int boidIndex, const int *&outIndices,
                               int &outCount) const {
  outIndices = nullptr;
  outCount = 0;

  if (!buffers_ || boidIndex < 0 ||
      boidIndex >= static_cast<int>(boidToLeafIndex_.size())) {
    return false;
  }

  const int leafIndex = boidToLeafIndex_[boidIndex];
  if (leafIndex < 0 || leafIndex >= static_cast<int>(leafRecords_.size())) {
    return false;
  }

  const LeafRecord &record = leafRecords_[leafIndex];
  if (record.count <= 0) {
    return false;
  }

  outIndices = leafIndexStorage_.data() + record.offset;
  outCount = record.count;
  return true;
}

int LbvhIndex::buildRecursive(int start, int end) {
  Node node;
  const int nodeIndex = static_cast<int>(nodes_.size());
  nodes_.push_back(node);

  Node &current = nodes_.back();

  const int count = end - start;
  if (count <= maxLeafSize_) {
    current.isLeaf = true;
    current.begin = static_cast<int>(leafIndexStorage_.size());
    current.end = current.begin + count;
    glm::vec3 boundsMin(std::numeric_limits<float>::max());
    glm::vec3 boundsMax(-std::numeric_limits<float>::max());
    for (int i = start; i < end; ++i) {
      const int boidIdx = mortonOrder_[i];
      leafIndexStorage_.push_back(boidIdx);
      const glm::vec3 &pos = buffers_->positions[boidIdx];
      boundsMin = vecMin(boundsMin, pos);
      boundsMax = vecMax(boundsMax, pos);
    }
    current.boundsMin = boundsMin;
    current.boundsMax = boundsMax;
    const int recordIndex = static_cast<int>(leafRecords_.size());
    leafRecords_.push_back(LeafRecord{nodeIndex, current.begin, count});
    for (int i = current.begin; i < current.end; ++i) {
      const int boidIdx = leafIndexStorage_[i];
      if (boidIdx >= 0 &&
          boidIdx < static_cast<int>(boidToLeafIndex_.size())) {
        boidToLeafIndex_[boidIdx] = recordIndex;
      }
    }
    return nodeIndex;
  }

  const int mid = (start + end) / 2;
  current.left = buildRecursive(start, mid);
  current.right = buildRecursive(mid, end);
  current.isLeaf = false;

  current.boundsMin = vecMin(nodes_[current.left].boundsMin,
                             nodes_[current.right].boundsMin);
  current.boundsMax = vecMax(nodes_[current.left].boundsMax,
                             nodes_[current.right].boundsMax);

  return nodeIndex;
}

glm::vec3 LbvhIndex::vecMin(const glm::vec3 &a, const glm::vec3 &b) {
  return glm::vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

glm::vec3 LbvhIndex::vecMax(const glm::vec3 &a, const glm::vec3 &b) {
  return glm::vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

void LbvhIndex::resetQueryStats() const {
  stats_.queries.store(0, std::memory_order_relaxed);
  stats_.nodesVisited.store(0, std::memory_order_relaxed);
  stats_.leavesVisited.store(0, std::memory_order_relaxed);
  stats_.boidsConsidered.store(0, std::memory_order_relaxed);
  stats_.maxQueueSize.store(0, std::memory_order_relaxed);
}

LbvhIndex::QueryStats LbvhIndex::consumeQueryStats() const {
  QueryStats snapshot;
  snapshot.queries = stats_.queries.exchange(0, std::memory_order_relaxed);
  snapshot.nodesVisited =
      stats_.nodesVisited.exchange(0, std::memory_order_relaxed);
  snapshot.leavesVisited =
      stats_.leavesVisited.exchange(0, std::memory_order_relaxed);
  snapshot.boidsConsidered =
      stats_.boidsConsidered.exchange(0, std::memory_order_relaxed);
  snapshot.maxQueueSize =
      stats_.maxQueueSize.exchange(0, std::memory_order_relaxed);
  return snapshot;
}
