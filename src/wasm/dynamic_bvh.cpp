#include "dynamic_bvh.h"
#include <algorithm>
#include <limits>

// SAH を用いた兄弟選択を行うルーズ BVH 実装。

DynamicBVH::DynamicBVH() {
  nodes_.reserve(128);
  queryStack_.reserve(128);
}

void DynamicBVH::clear() {
  nodes_.clear();
  queryStack_.clear();
  root_ = -1;
  freeList_ = -1;
  nodeCount_ = 0;
}

int DynamicBVH::allocateNode() {
  int nodeId;
  if (freeList_ != -1) {
    nodeId = freeList_;
    freeList_ = nodes_[nodeId].next;
  } else {
    nodeId = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{});
  }
  nodes_[nodeId] = Node{};
  nodeCount_++;
  return nodeId;
}

void DynamicBVH::freeNode(int nodeId) {
  nodes_[nodeId].next = freeList_;
  freeList_ = nodeId;
  nodeCount_--;
}

int DynamicBVH::createProxy(const glm::vec3 &min, const glm::vec3 &max,
                            int boidIndex) {
  int nodeId = allocateNode();
  Node &node = nodes_[nodeId];
  node.aabbMin = min;
  node.aabbMax = max;
  node.boidIndex = boidIndex;
  insertLeaf(nodeId);
  return nodeId;
}

void DynamicBVH::destroyProxy(int proxyId) {
  if (proxyId < 0)
    return;
  removeLeaf(proxyId);
  freeNode(proxyId);
}

bool DynamicBVH::updateProxy(int proxyId, const glm::vec3 &min,
                             const glm::vec3 &max,
                             const glm::vec3 &fatMargin) {
  Node &node = nodes_[proxyId];
  if (node.aabbMin.x <= min.x && node.aabbMin.y <= min.y &&
      node.aabbMin.z <= min.z && node.aabbMax.x >= max.x &&
      node.aabbMax.y >= max.y && node.aabbMax.z >= max.z) {
    return false; // still inside fat AABB
  }

  removeLeaf(proxyId);
  node.aabbMin = min - fatMargin;
  node.aabbMax = max + fatMargin;
  insertLeaf(proxyId);
  return true;
}

void DynamicBVH::insertLeaf(int leafId) {
  if (root_ == -1) {
    root_ = leafId;
    nodes_[root_].parent = -1;
    return;
  }

  Node &leaf = nodes_[leafId];
  int siblingId = pickBestSibling(leaf.aabbMin, leaf.aabbMax);

  int oldParent = nodes_[siblingId].parent;
  int newParent = allocateNode();
  Node &parentNode = nodes_[newParent];
  parentNode.parent = oldParent;
  parentNode.child1 = siblingId;
  parentNode.child2 = leafId;
  parentNode.aabbMin = combineMin(leaf.aabbMin, nodes_[siblingId].aabbMin);
  parentNode.aabbMax = combineMax(leaf.aabbMax, nodes_[siblingId].aabbMax);
  parentNode.boidIndex = -1;

  nodes_[siblingId].parent = newParent;
  leaf.parent = newParent;

  if (oldParent != -1) {
    Node &oldParentNode = nodes_[oldParent];
    if (oldParentNode.child1 == siblingId) {
      oldParentNode.child1 = newParent;
    } else {
      oldParentNode.child2 = newParent;
    }
  } else {
    root_ = newParent;
  }

  fixUpwardsTree(newParent);
}

void DynamicBVH::removeLeaf(int leafId) {
  if (leafId == root_) {
    root_ = -1;
    return;
  }

  int parent = nodes_[leafId].parent;
  int grandParent = nodes_[parent].parent;
  int sibling = nodes_[parent].child1 == leafId ? nodes_[parent].child2
                                                : nodes_[parent].child1;

  if (grandParent != -1) {
    Node &gp = nodes_[grandParent];
    if (gp.child1 == parent) {
      gp.child1 = sibling;
    } else {
      gp.child2 = sibling;
    }
    nodes_[sibling].parent = grandParent;
    freeNode(parent);
    fixUpwardsTree(grandParent);
  } else {
    root_ = sibling;
    nodes_[sibling].parent = -1;
    freeNode(parent);
  }

  nodes_[leafId].parent = -1;
}

void DynamicBVH::fixUpwardsTree(int nodeId) {
  while (nodeId != -1) {
    Node &node = nodes_[nodeId];
    const Node &child1 = nodes_[node.child1];
    const Node &child2 = nodes_[node.child2];
    node.aabbMin = combineMin(child1.aabbMin, child2.aabbMin);
    node.aabbMax = combineMax(child1.aabbMax, child2.aabbMax);
    nodeId = node.parent;
  }
}

int DynamicBVH::pickBestSibling(const glm::vec3 &leafMin,
                                const glm::vec3 &leafMax) const {
  int index = root_;
  while (!nodes_[index].isLeaf()) {
    const Node &node = nodes_[index];
    const Node &child1 = nodes_[node.child1];
    const Node &child2 = nodes_[node.child2];

    glm::vec3 combinedMin1 = combineMin(node.aabbMin, leafMin);
    glm::vec3 combinedMax1 = combineMax(node.aabbMax, leafMax);
    float combinedArea = surfaceArea(combinedMin1, combinedMax1);
    float cost = 2.0f * combinedArea;

    float inheritedArea = 2.0f * (combinedArea - surfaceArea(node.aabbMin, node.aabbMax));

    float cost1;
    if (child1.isLeaf()) {
      glm::vec3 min1 = combineMin(child1.aabbMin, leafMin);
      glm::vec3 max1 = combineMax(child1.aabbMax, leafMax);
      cost1 = surfaceArea(min1, max1) + inheritedArea;
    } else {
      glm::vec3 min1 = combineMin(child1.aabbMin, leafMin);
      glm::vec3 max1 = combineMax(child1.aabbMax, leafMax);
      float oldArea = surfaceArea(child1.aabbMin, child1.aabbMax);
      float newArea = surfaceArea(min1, max1);
      cost1 = (newArea - oldArea) + inheritedArea;
    }

    float cost2;
    if (child2.isLeaf()) {
      glm::vec3 min2 = combineMin(child2.aabbMin, leafMin);
      glm::vec3 max2 = combineMax(child2.aabbMax, leafMax);
      cost2 = surfaceArea(min2, max2) + inheritedArea;
    } else {
      glm::vec3 min2 = combineMin(child2.aabbMin, leafMin);
      glm::vec3 max2 = combineMax(child2.aabbMax, leafMax);
      float oldArea = surfaceArea(child2.aabbMin, child2.aabbMax);
      float newArea = surfaceArea(min2, max2);
      cost2 = (newArea - oldArea) + inheritedArea;
    }

    if (cost < cost1 && cost < cost2)
      break;

    index = (cost1 < cost2) ? node.child1 : node.child2;
  }

  return index;
}

bool DynamicBVH::overlaps(const glm::vec3 &aMin, const glm::vec3 &aMax,
                          const glm::vec3 &bMin, const glm::vec3 &bMax) {
  if (aMax.x < bMin.x || aMin.x > bMax.x)
    return false;
  if (aMax.y < bMin.y || aMin.y > bMax.y)
    return false;
  if (aMax.z < bMin.z || aMin.z > bMax.z)
    return false;
  return true;
}

glm::vec3 DynamicBVH::combineMin(const glm::vec3 &aMin,
                                 const glm::vec3 &bMin) {
  return glm::vec3(std::min(aMin.x, bMin.x), std::min(aMin.y, bMin.y),
                   std::min(aMin.z, bMin.z));
}

glm::vec3 DynamicBVH::combineMax(const glm::vec3 &aMax,
                                 const glm::vec3 &bMax) {
  return glm::vec3(std::max(aMax.x, bMax.x), std::max(aMax.y, bMax.y),
                   std::max(aMax.z, bMax.z));
}

float DynamicBVH::surfaceArea(const glm::vec3 &min, const glm::vec3 &max) {
  glm::vec3 extents = max - min;
  return 2.0f * (extents.x * extents.y + extents.y * extents.z +
                 extents.z * extents.x);
}
