#include "dbvh_index.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stack>
#include <thread>

#include <glm/glm.hpp>

namespace {
constexpr float kEpsilon = 1e-5f;
constexpr int kMaxQueryStackReserve = 256;

struct QueryScratch {
  std::vector<int> stack;
};

inline QueryScratch &threadScratch() {
  thread_local QueryScratch scratch{};
  if (scratch.stack.capacity() < kMaxQueryStackReserve) {
    scratch.stack.reserve(kMaxQueryStackReserve);
  }
  return scratch;
}

inline glm::vec3 makeFatExtent(float radius) {
  return glm::vec3(radius);
}
} // namespace

void DbvhIndex::clear() {
  nodes_.clear();
  handles_.clear();
  root_ = -1;
  freeList_ = -1;
  buffers_ = nullptr;
}

void DbvhIndex::sync(const SoABuffers &buffers, float baseRadius,
                     float velocityPadding) {
  const std::size_t count = buffers.positions.size();
  const float clampedBase = std::max(baseRadius, kEpsilon);
  const float fatRadius = std::max(clampedBase + velocityPadding, clampedBase);

  if (count == 0) {
    clear();
    return;
  }

  const bool needRebuild = handles_.size() != count || nodes_.empty();
  if (needRebuild) {
    rebuild(buffers, fatRadius);
    return;
  }

  buffers_ = &buffers;
  for (std::size_t i = 0; i < count; ++i) {
    const int handle = handles_[i];
    if (handle < 0 || handle >= static_cast<int>(nodes_.size())) {
      rebuild(buffers, fatRadius);
      return;
    }
    updateLeaf(handle, buffers.positions[i], fatRadius);
  }
}

int DbvhIndex::gatherWithinRadius(const glm::vec3 &center, float radius,
                                  int *outIndices, int maxOut) const {
  if (root_ == -1 || !outIndices || maxOut <= 0 || radius <= 0.0f) {
    return 0;
  }
  QueryScratch &scratch = threadScratch();
  auto &stack = scratch.stack;
  stack.clear();
  stack.push_back(root_);

  const float radiusSq = radius * radius;
  int emitted = 0;
  while (!stack.empty()) {
    const int nodeIndex = stack.back();
    stack.pop_back();
    const Node &node = nodes_[nodeIndex];
    const float sq = distanceSqAabb(center, node.aabbMin, node.aabbMax);
    if (sq > radiusSq) {
      continue;
    }
    if (node.isLeaf()) {
      if (node.boidIndex >= 0) {
        outIndices[emitted++] = node.boidIndex;
        if (emitted >= maxOut) {
          break;
        }
      }
      continue;
    }
    if (node.left >= 0) {
      stack.push_back(node.left);
    }
    if (node.right >= 0) {
      stack.push_back(node.right);
    }
  }
  return emitted;
}

void DbvhIndex::rebuild(const SoABuffers &buffers, float fatRadius) {
  clear();
  buffers_ = &buffers;
  ensureCapacity(buffers.positions.size() * 2u);

  handles_.assign(buffers.positions.size(), -1);
  const glm::vec3 extent = makeFatExtent(fatRadius);
  for (std::size_t i = 0; i < buffers.positions.size(); ++i) {
    const glm::vec3 &pos = buffers.positions[i];
    const int leaf = allocateNode();
    Node &node = nodes_[leaf];
    node.aabbMin = pos - extent;
    node.aabbMax = pos + extent;
    node.height = 0;
    node.boidIndex = static_cast<int>(i);
    node.parent = -1;
    node.left = -1;
    node.right = -1;
    insertLeaf(leaf);
    handles_[i] = leaf;
  }
}

void DbvhIndex::ensureCapacity(std::size_t count) {
  if (nodes_.capacity() < count) {
    nodes_.reserve(count);
  }
}

int DbvhIndex::allocateNode() {
  if (freeList_ != -1) {
    const int nodeIndex = freeList_;
    Node &node = nodes_[nodeIndex];
    freeList_ = node.next;
    node.parent = -1;
    node.left = -1;
    node.right = -1;
    node.height = 0;
    node.next = -1;
    node.boidIndex = -1;
    return nodeIndex;
  }
  Node node;
  nodes_.push_back(node);
  return static_cast<int>(nodes_.size() - 1);
}

void DbvhIndex::freeNode(int nodeIndex) {
  Node &node = nodes_[nodeIndex];
  node.next = freeList_;
  node.height = -1;
  node.parent = -1;
  node.left = -1;
  node.right = -1;
  node.boidIndex = -1;
  freeList_ = nodeIndex;
}

void DbvhIndex::insertLeaf(int leafIndex) {
  if (root_ == -1) {
    root_ = leafIndex;
    nodes_[root_].parent = -1;
    return;
  }

  int sibling = pickBestSibling(leafIndex);

  const int oldParent = nodes_[sibling].parent;
  const int newParent = allocateNode();
  Node &parentNode = nodes_[newParent];
  parentNode.parent = oldParent;
  parentNode.left = sibling;
  parentNode.right = leafIndex;
  parentNode.height = nodes_[sibling].height + 1;
  parentNode.aabbMin = combineMin(nodes_[sibling].aabbMin, nodes_[leafIndex].aabbMin);
  parentNode.aabbMax = combineMax(nodes_[sibling].aabbMax, nodes_[leafIndex].aabbMax);
  parentNode.boidIndex = -1;

  nodes_[sibling].parent = newParent;
  nodes_[leafIndex].parent = newParent;

  if (oldParent != -1) {
    Node &oldParentNode = nodes_[oldParent];
    if (oldParentNode.left == sibling) {
      oldParentNode.left = newParent;
    } else {
      oldParentNode.right = newParent;
    }
  } else {
    root_ = newParent;
    parentNode.parent = -1;
  }

  int index = nodes_[leafIndex].parent;
  while (index != -1) {
    index = balance(index);

    Node &node = nodes_[index];
    Node &left = nodes_[node.left];
    Node &right = nodes_[node.right];

    node.aabbMin = combineMin(left.aabbMin, right.aabbMin);
    node.aabbMax = combineMax(left.aabbMax, right.aabbMax);
    node.height = 1 + std::max(left.height, right.height);

    index = node.parent;
  }
}

void DbvhIndex::removeLeaf(int leafIndex) {
  if (leafIndex == root_) {
    root_ = -1;
    return;
  }

  const int parent = nodes_[leafIndex].parent;
  const int grandParent = nodes_[parent].parent;
  int sibling;
  if (nodes_[parent].left == leafIndex) {
    sibling = nodes_[parent].right;
  } else {
    sibling = nodes_[parent].left;
  }

  if (grandParent != -1) {
    Node &grandNode = nodes_[grandParent];
    if (grandNode.left == parent) {
      grandNode.left = sibling;
    } else {
      grandNode.right = sibling;
    }
    nodes_[sibling].parent = grandParent;
    freeNode(parent);
    fixUpwardsTree(grandParent);
  } else {
    root_ = sibling;
    nodes_[sibling].parent = -1;
    freeNode(parent);
  }
  nodes_[leafIndex].parent = -1;
}

void DbvhIndex::updateLeaf(int leafIndex, const glm::vec3 &pos,
                           float fatRadius) {
  if (leafIndex < 0 || leafIndex >= static_cast<int>(nodes_.size())) {
    return;
  }
  Node &node = nodes_[leafIndex];
  const glm::vec3 extent = makeFatExtent(fatRadius);
  const glm::vec3 newMin = pos - extent;
  const glm::vec3 newMax = pos + extent;

  if (contains(node, newMin, newMax)) {
    return;
  }

  removeLeaf(leafIndex);
  node.aabbMin = newMin;
  node.aabbMax = newMax;
  node.height = 0;
  insertLeaf(leafIndex);
}

int DbvhIndex::pickBestSibling(int leafIndex) const {
  const glm::vec3 &leafMin = nodes_[leafIndex].aabbMin;
  const glm::vec3 &leafMax = nodes_[leafIndex].aabbMax;
  int index = root_;
  while (!nodes_[index].isLeaf()) {
    const int left = nodes_[index].left;
    const int right = nodes_[index].right;

    const glm::vec3 combinedMin = combineMin(nodes_[index].aabbMin, leafMin);
    const glm::vec3 combinedMax = combineMax(nodes_[index].aabbMax, leafMax);
    const float combinedArea = surfaceArea(combinedMin, combinedMax);
    const float currentArea = surfaceArea(nodes_[index].aabbMin, nodes_[index].aabbMax);
    const float inheritanceCost = 2.0f * (combinedArea - currentArea);

    float costLeft;
    if (nodes_[left].isLeaf()) {
      const glm::vec3 minL = combineMin(nodes_[left].aabbMin, leafMin);
      const glm::vec3 maxL = combineMax(nodes_[left].aabbMax, leafMax);
      costLeft = surfaceArea(minL, maxL) + inheritanceCost;
    } else {
      const glm::vec3 minL = combineMin(nodes_[left].aabbMin, leafMin);
      const glm::vec3 maxL = combineMax(nodes_[left].aabbMax, leafMax);
      const float oldArea = surfaceArea(nodes_[left].aabbMin, nodes_[left].aabbMax);
      costLeft = (surfaceArea(minL, maxL) - oldArea) + inheritanceCost;
    }

    float costRight;
    if (nodes_[right].isLeaf()) {
      const glm::vec3 minR = combineMin(nodes_[right].aabbMin, leafMin);
      const glm::vec3 maxR = combineMax(nodes_[right].aabbMax, leafMax);
      costRight = surfaceArea(minR, maxR) + inheritanceCost;
    } else {
      const glm::vec3 minR = combineMin(nodes_[right].aabbMin, leafMin);
      const glm::vec3 maxR = combineMax(nodes_[right].aabbMax, leafMax);
      const float oldArea = surfaceArea(nodes_[right].aabbMin, nodes_[right].aabbMax);
      costRight = (surfaceArea(minR, maxR) - oldArea) + inheritanceCost;
    }

    if (costLeft < costRight) {
      index = left;
    } else {
      index = right;
    }
  }
  return index;
}

int DbvhIndex::balance(int index) {
  Node &nodeA = nodes_[index];
  if (nodeA.isLeaf() || nodeA.height < 2) {
    return index;
  }

  const int left = nodeA.left;
  const int right = nodeA.right;
  Node &nodeB = nodes_[left];
  Node &nodeC = nodes_[right];

  const int balanceFactor = nodeC.height - nodeB.height;

  if (balanceFactor > 1) {
    const int childLeft = nodeC.left;
    const int childRight = nodeC.right;
    if (childLeft == -1 || childRight == -1) {
      return index;
    }
    Node &nodeF = nodes_[childLeft];
    Node &nodeG = nodes_[childRight];

    nodeC.left = index;
    nodeC.parent = nodeA.parent;
    nodeA.parent = right;

    if (nodeC.parent != -1) {
      Node &parent = nodes_[nodeC.parent];
      if (parent.left == index) {
        parent.left = right;
      } else {
        parent.right = right;
      }
    } else {
      root_ = right;
    }

    if (nodeF.height > nodeG.height) {
      nodeC.right = childLeft;
      nodeA.right = childRight;
      nodeG.parent = index;
      nodeA.aabbMin = combineMin(nodeB.aabbMin, nodeG.aabbMin);
      nodeA.aabbMax = combineMax(nodeB.aabbMax, nodeG.aabbMax);
      nodeC.aabbMin = combineMin(nodeA.aabbMin, nodeF.aabbMin);
      nodeC.aabbMax = combineMax(nodeA.aabbMax, nodeF.aabbMax);
      nodeA.height = 1 + std::max(nodeB.height, nodeG.height);
      nodeC.height = 1 + std::max(nodeA.height, nodeF.height);
    } else {
      nodeC.right = childRight;
      nodeA.right = childLeft;
      nodeF.parent = index;
      nodeA.aabbMin = combineMin(nodeB.aabbMin, nodeF.aabbMin);
      nodeA.aabbMax = combineMax(nodeB.aabbMax, nodeF.aabbMax);
      nodeC.aabbMin = combineMin(nodeA.aabbMin, nodeG.aabbMin);
      nodeC.aabbMax = combineMax(nodeA.aabbMax, nodeG.aabbMax);
      nodeA.height = 1 + std::max(nodeB.height, nodeF.height);
      nodeC.height = 1 + std::max(nodeA.height, nodeG.height);
    }
    return right;
  }

  if (balanceFactor < -1) {
    const int childLeft = nodeB.left;
    const int childRight = nodeB.right;
    if (childLeft == -1 || childRight == -1) {
      return index;
    }
    Node &nodeD = nodes_[childLeft];
    Node &nodeE = nodes_[childRight];

    nodeB.left = index;
    nodeB.parent = nodeA.parent;
    nodeA.parent = left;

    if (nodeB.parent != -1) {
      Node &parent = nodes_[nodeB.parent];
      if (parent.left == index) {
        parent.left = left;
      } else {
        parent.right = left;
      }
    } else {
      root_ = left;
    }

    if (nodeD.height > nodeE.height) {
      nodeB.right = childLeft;
      nodeA.left = childRight;
      nodeE.parent = index;
      nodeA.aabbMin = combineMin(nodeC.aabbMin, nodeE.aabbMin);
      nodeA.aabbMax = combineMax(nodeC.aabbMax, nodeE.aabbMax);
      nodeB.aabbMin = combineMin(nodeA.aabbMin, nodeD.aabbMin);
      nodeB.aabbMax = combineMax(nodeA.aabbMax, nodeD.aabbMax);
      nodeA.height = 1 + std::max(nodeC.height, nodeE.height);
      nodeB.height = 1 + std::max(nodeA.height, nodeD.height);
    } else {
      nodeB.right = childRight;
      nodeA.left = childLeft;
      nodeD.parent = index;
      nodeA.aabbMin = combineMin(nodeC.aabbMin, nodeD.aabbMin);
      nodeA.aabbMax = combineMax(nodeC.aabbMax, nodeD.aabbMax);
      nodeB.aabbMin = combineMin(nodeA.aabbMin, nodeE.aabbMin);
      nodeB.aabbMax = combineMax(nodeA.aabbMax, nodeE.aabbMax);
      nodeA.height = 1 + std::max(nodeC.height, nodeD.height);
      nodeB.height = 1 + std::max(nodeA.height, nodeE.height);
    }
    return left;
  }

  return index;
}

void DbvhIndex::fixUpwardsTree(int index) {
  while (index != -1) {
    index = balance(index);
    Node &node = nodes_[index];
    if (node.left >= 0 && node.right >= 0) {
      Node &left = nodes_[node.left];
      Node &right = nodes_[node.right];
      node.aabbMin = combineMin(left.aabbMin, right.aabbMin);
      node.aabbMax = combineMax(left.aabbMax, right.aabbMax);
      node.height = 1 + std::max(left.height, right.height);
    }
    index = node.parent;
  }
}

glm::vec3 DbvhIndex::combineMin(const glm::vec3 &a, const glm::vec3 &b) {
  return glm::vec3(std::min(a.x, b.x), std::min(a.y, b.y),
                   std::min(a.z, b.z));
}

glm::vec3 DbvhIndex::combineMax(const glm::vec3 &a, const glm::vec3 &b) {
  return glm::vec3(std::max(a.x, b.x), std::max(a.y, b.y),
                   std::max(a.z, b.z));
}

float DbvhIndex::surfaceArea(const glm::vec3 &min, const glm::vec3 &max) {
  const glm::vec3 extents = max - min;
  return 2.0f * (extents.x * extents.y + extents.y * extents.z +
                 extents.z * extents.x);
}

bool DbvhIndex::contains(const Node &node, const glm::vec3 &min,
                         const glm::vec3 &max) {
  return min.x >= node.aabbMin.x && min.y >= node.aabbMin.y &&
         min.z >= node.aabbMin.z && max.x <= node.aabbMax.x &&
         max.y <= node.aabbMax.y && max.z <= node.aabbMax.z;
}

float DbvhIndex::distanceSqAabb(const glm::vec3 &point, const glm::vec3 &min,
                                const glm::vec3 &max) {
  float distSq = 0.0f;
  const float comps[3] = {point.x, point.y, point.z};
  const float mins[3] = {min.x, min.y, min.z};
  const float maxs[3] = {max.x, max.y, max.z};
  for (int i = 0; i < 3; ++i) {
    if (comps[i] < mins[i]) {
      const float d = mins[i] - comps[i];
      distSq += d * d;
    } else if (comps[i] > maxs[i]) {
      const float d = comps[i] - maxs[i];
      distSq += d * d;
    }
  }
  return distSq;
}
