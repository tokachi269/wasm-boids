#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"
#include "boid_factory.h"
#include "vec3.h"
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <random>
#include <iostream>
#include "species_params.h"
#include "boid.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>

// グローバル共通
SpeciesParams globalSpeciesParams;

SpeciesParams getGlobalSpeciesParams()
{
    return globalSpeciesParams;
}
void setGlobalSpeciesParams(const SpeciesParams &params)
{
    globalSpeciesParams = params;
}
BoidTree::BoidTree()
    : root(nullptr), frameCount(0), splitIndex(0), mergeIndex(0), maxBoidsPerUnit(10)
{
}

void printTree(const BoidUnit *node, int depth)
{
    if (!node)
        return;
    std::string indent(depth * 2, ' ');
    std::cout << indent << "Level: " << node->level
              << " | Boids: " << node->ids.size()
              << " | Children: " << node->children.size()
              << " | Center: (" << node->center.x << ", " << node->center.y << ", " << node->center.z << ")"
              << " | Radius: " << node->radius << std::endl;
    for (const auto *child : node->children)
    {
        printTree(child, depth + 1);
    }
}

// BoidTreeのメンバ関数実装
void BoidTree::build(int maxPerUnit, int level) {
    if (!root) {
        root = new BoidUnit();
    }

    root->level = level;

    // すべてのインデックスを初期化
    std::vector<int> indices(positions.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = static_cast<int>(i);
    }

    // デバッグログ
    std::cout << "Building tree with " << indices.size() << " boids." << std::endl;

    buildRecursive(root, indices, maxPerUnit, level);
}

void BoidTree::buildRecursive(BoidUnit* node, const std::vector<int>& indices, int maxPerUnit, int level) {
    node->level = level;

    if ((int)indices.size() <= maxPerUnit) {
        for (int i : indices) {
            node->positions.push_back(positions[i]);
            node->velocities.push_back(velocities[i]);
            node->accelerations.push_back(glm::vec3(0.0f));
            node->ids.push_back(ids[i]);
            node->stresses.push_back(stresses[i]);
            node->speciesIds.push_back(speciesIds[i]);
            node->cohesionMemories[ids[i]] = cohesionMemories[ids[i]];
        }
        node->computeBoundingSphere();
        return;
    }

    float mean[3] = {0}, var[3] = {0};
    for (int i : indices) {
        const glm::vec3& pos = positions[i];
        mean[0] += pos.x;
        mean[1] += pos.y;
        mean[2] += pos.z;
    }
    for (int i = 0; i < 3; ++i) mean[i] /= indices.size();

    for (int i : indices) {
        const glm::vec3& pos = positions[i];
        var[0] += (pos.x - mean[0]) * (pos.x - mean[0]);
        var[1] += (pos.y - mean[1]) * (pos.y - mean[1]);
        var[2] += (pos.z - mean[2]) * (pos.z - mean[2]);
    }

    int axis = (var[1] > var[0]) ? 1 : 0;
    if (var[2] > var[axis]) axis = 2;

    std::vector<int> sorted = indices;
    std::sort(sorted.begin(), sorted.end(), [this, axis](int a, int b) {
        const glm::vec3& pa = positions[a];
        const glm::vec3& pb = positions[b];
        return (axis == 0) ? pa.x < pb.x : (axis == 1) ? pa.y < pb.y : pa.z < pb.z;
    });

    size_t mid = sorted.size() / 2;
    std::vector<int> left(sorted.begin(), sorted.begin() + mid);
    std::vector<int> right(sorted.begin() + mid, sorted.end());

    BoidUnit* leftChild = new BoidUnit();
    BoidUnit* rightChild = new BoidUnit();
    node->children = {leftChild, rightChild};

    buildRecursive(leftChild, left, maxPerUnit, level + 1);
    buildRecursive(rightChild, right, maxPerUnit, level + 1);

    node->computeBoundingSphere();
}

BoidUnit *BoidTree::findParent(BoidUnit *node, BoidUnit *target)
{
    if (!node || node->children.empty())
        return nullptr;
    for (auto *c : node->children)
    {
        if (!c)
            continue;
        if (c == target)
            return node;
        BoidUnit *res = findParent(c, target);
        if (res)
            return res;
    }
    return nullptr;
}

void BoidTree::update(float dt)
{
    // 木構造全体を再帰的に更新
    if (root)
    {
        root->updateRecursive(dt);
    }

    frameCount++;

    // 一定フレームごとに葉ノードを再収集
    if (frameCount % 5 == 0)
    {
        leafCache.clear();
        collectLeaves(root, leafCache);
        splitIndex = 0;
        mergeIndex = 0;
    }

    // 一定フレームごとに木構造を再構築
    if (frameCount % 170 == 0)
    { // 再構築頻度を調整
        build(maxBoidsPerUnit, 0);
        return;
    }

    // 分割と結合の処理
    if (!leafCache.empty())
    {
        // 分割
        for (int i = 0; i < 5 && splitIndex < (int)leafCache.size(); ++i, ++splitIndex)
        {
            BoidUnit *u = leafCache[splitIndex];
            if (u && u->needsSplit(40.0f, 0.5f, maxBoidsPerUnit))
            {
                u->splitInPlace(maxBoidsPerUnit);
                leafCache.clear();
                collectLeaves(root, leafCache);
                break;
            }
        }

        // 結合
        for (int i = 0; i < 5 && mergeIndex < (int)leafCache.size(); ++i, ++mergeIndex)
        {
            for (int j = mergeIndex + 1; j < (int)leafCache.size(); ++j)
            {
                BoidUnit *a = leafCache[mergeIndex];
                BoidUnit *b = leafCache[j];
                if (a && b && a->canMergeWith(*b))
                {
                    BoidUnit *parent = findParent(root, b);
                    if (parent)
                    {
                        leafCache.clear();
                        a->mergeWith(b, parent);
                        collectLeaves(root, leafCache);
                    }
                    break;
                }
            }
        }
    }
}

// 分割判定を局所的に適用
void BoidTree::trySplitRecursive(BoidUnit *node)
{
    if (!node)
        return;
    if (node->isBoidUnit() && node->needsSplit(40.0f, 0.5f, maxBoidsPerUnit))
    {
        node->splitInPlace(maxBoidsPerUnit);
    }
    for (auto *c : node->children)
        trySplitRecursive(c);
}

std::vector<Boid> BoidTree::getBoids() const
{
    std::vector<Boid> result;
    collectBoids(root, result);
    return result;
}

void BoidTree::collectBoids(const BoidUnit *node, std::vector<Boid> &result) const
{
    if (!node)
        return;
    if (node->isBoidUnit())
    {
        for (int index : node->ids)
        {
            Boid b;
            b.position = root->positions[index];
            b.velocity = root->velocities[index];
            b.acceleration = root->accelerations[index];
            b.id = root->ids[index];
            b.stress = root->stresses[index];
            b.speciesId = root->speciesIds[index];
            result.push_back(b);
        }
    }
    else
    {
        for (const auto *child : node->children)
        {
            collectBoids(child, result);
        }
    }
}

void BoidTree::collectLeaves(const BoidUnit *node, std::vector<BoidUnit *> &leaves) const
{
    if (!node)
        return;
    // children配列の中身がnullptrでないかチェック
    if (node->isBoidUnit())
    {
        leaves.push_back(const_cast<BoidUnit *>(node));
    }
    else
    {
        for (const auto *child : node->children)
        {
            if (child)
                collectLeaves(child, leaves);
        }
    }
}

void BoidTree::initializeBoids(int count, float posRange, float velRange)
{
    positions.resize(count);
    velocities.resize(count);
    accelerations.resize(count, glm::vec3(0.0f));
    ids.resize(count);
    stresses.resize(count, 0.0f);
    speciesIds.resize(count, 0);
    for (int i = 0; i < count; ++i) {
        cohesionMemories[i] = std::unordered_map<int, float>();
    }
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int i = 0; i < count; ++i)
    {
        positions[i] = glm::vec3(posDist(gen), posDist(gen), posDist(gen));
        velocities[i] = glm::vec3(velDist(gen), velDist(gen), velDist(gen));
        ids[i] = i;
    }

    if (!root) root = new BoidUnit();

    // build() は中央バッファをもとに木構造を作る
    std::vector<int> indices(count);
    std::iota(indices.begin(), indices.end(), 0);  // 0, 1, ..., count-1

    buildRecursive(root, indices, maxBoidsPerUnit, 0);
    printTree(root);
}

void BoidTree::setFlockSize(int newSize, float posRange, float velRange)
{
    int current = static_cast<int>(root->positions.size());

    if (newSize < current)
    {
        root->positions.resize(newSize);
        root->velocities.resize(newSize);
        root->accelerations.resize(newSize);
        root->ids.resize(newSize);
        root->stresses.resize(newSize);
        root->speciesIds.resize(newSize);
    }
    else if (newSize > current)
    {
        auto added = BoidFactory::generateRandomBoids(newSize - current, posRange, velRange);

        for (const auto &b : added)
        {
            root->positions.push_back(b.position);
            root->velocities.push_back(b.velocity);
            root->accelerations.push_back(b.acceleration);
            root->ids.push_back(b.id);
            root->stresses.push_back(b.stress);
            root->speciesIds.push_back(b.speciesId);
        }
    }

    build(maxBoidsPerUnit, 0);
}

void BoidTree::updatePositionBuffer()
{
    positionBuffer.clear();
    std::vector<BoidUnit *> leaves;
    collectLeaves(root, leaves); // すべての leaf node を収集

    for (BoidUnit *leaf : leaves)
    {
        for (const glm::vec3 &pos : leaf->positions)
        {
            positionBuffer.push_back(pos.x);
            positionBuffer.push_back(pos.y);
            positionBuffer.push_back(pos.z);
        }
    }
}

uintptr_t BoidTree::getPositionBufferPtr()
{
    return reinterpret_cast<uintptr_t>(positionBuffer.data());
}

int BoidTree::getBoidCount() const
{
    return (int)positionBuffer.size() / 3;
}

void BoidTree::updateVelocityBuffer()
{
    velocityBuffer.clear();
    std::vector<BoidUnit *> leaves;
    collectLeaves(root, leaves);

    for (BoidUnit *leaf : leaves)
    {
        for (const glm::vec3 &vel : leaf->velocities)
        {
            velocityBuffer.push_back(vel.x);
            velocityBuffer.push_back(vel.y);
            velocityBuffer.push_back(vel.z);
        }
    }
}

uintptr_t BoidTree::getVelocityBufferPtr()
{
    return reinterpret_cast<uintptr_t>(velocityBuffer.data());
}