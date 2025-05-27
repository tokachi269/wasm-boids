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

void printTree(const BoidUnit *node, int depth)
{
    if (!node)
        return;
    std::string indent(depth * 2, ' ');
    std::cout << indent << "Level: " << node->level
              << " | Boids: " << node->boids.size()
              << " | Children: " << node->children.size()
              << " | Center: (" << node->center.x << ", " << node->center.y << ", " << node->center.z << ")"
              << " | Radius: " << node->radius << std::endl;
    for (const auto *child : node->children)
    {
        printTree(child, depth + 1);
    }
}

// BoidTreeのメンバ関数実装
BoidTree::BoidTree() : root(nullptr) {}

void BoidTree::build(int maxPerUnit, int level)
{
    std::cout << "Building BoidTree with maxPerUnit: " << maxPerUnit << " at level: " << level << std::endl;
    maxBoidsPerUnit = maxPerUnit;
    if (!root) {
        root = new BoidUnit(); // 初回のみ新しいルートノードを作成
    }
    root->level = level;
    buildRecursive(root, root->boids, maxPerUnit, level);
}

void BoidTree::buildRecursive(BoidUnit *node, std::vector<Boid> &boids, int maxPerUnit, int level)
{
    if ((int)boids.size() <= maxPerUnit)
    {
        node->boids = boids;
        node->computeBoundingSphere();
        return;
    }
    // --- 最大分散軸を求める ---
    float mean[3] = {0, 0, 0}, var[3] = {0, 0, 0};
    for (const auto &b : boids)
    {
        mean[0] += b.position.x;
        mean[1] += b.position.y;
        mean[2] += b.position.z;
    }
    mean[0] /= boids.size();
    mean[1] /= boids.size();
    mean[2] /= boids.size();
    for (const auto &b : boids)
    {
        var[0] += (b.position.x - mean[0]) * (b.position.x - mean[0]);
        var[1] += (b.position.y - mean[1]) * (b.position.y - mean[1]);
        var[2] += (b.position.z - mean[2]) * (b.position.z - mean[2]);
    }
    int axis = 0;
    if (var[1] > var[0])
        axis = 1;
    if (var[2] > var[axis])
        axis = 2;

    std::sort(boids.begin(), boids.end(), [axis](const Boid &a, const Boid &b)
              {
        if (axis == 0) return a.position.x < b.position.x;
        if (axis == 1) return a.position.y < b.position.y;
        return a.position.z < b.position.z; });
    size_t mid = boids.size() / 2;
    std::vector<Boid> left(boids.begin(), boids.begin() + mid);
    std::vector<Boid> right(boids.begin() + mid, boids.end());
    BoidUnit *leftChild = new BoidUnit();
    BoidUnit *rightChild = new BoidUnit();
    leftChild->level = rightChild->level = level + 1;
    buildRecursive(leftChild, left, maxPerUnit, level + 1);
    buildRecursive(rightChild, right, maxPerUnit, level + 1);
    node->children = {leftChild, rightChild};
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
    // printTree(root);

    if (root)
        root->updateRecursive(dt);

    frameCount++;
    if (frameCount % 5 == 0)
    {
        leafCache.clear();
        collectLeaves(root, leafCache);
        splitIndex = 0;
        mergeIndex = 0;
    }
    // 再構築カウンタ
    if (frameCount % 17 == 0)
    {
        root->boids = getBoids();  // 最新化
        build(maxBoidsPerUnit, 0);       // 正しく再構築
        return;
    }

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
                        // ここでleafCacheをクリアしてからmergeWith
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
        result.insert(result.end(), node->boids.begin(), node->boids.end());
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

void BoidTree::initializeBoids(int count, float posRange, float velRange) {
    if (!root) {
        root = new BoidUnit();
    }
    // BoidFactory を使用して Boid を生成
    root->boids = BoidFactory::generateRandomBoids(count, posRange, velRange);
    build(maxBoidsPerUnit, 0);
}

void BoidTree::setFlockSize(int newSize, float posRange, float velRange) {
    int current = static_cast<int>(root->boids.size());

    if (newSize < current) {
        // ランダムに減らす
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(root->boids.begin(), root->boids.end(), gen);
        root->boids.resize(newSize);
    } else if (newSize > current) {
        // BoidFactory を使用して追加の Boid を生成
        auto added = BoidFactory::generateRandomBoids(newSize - current, posRange, velRange);
        root->boids.insert(root->boids.end(), added.begin(), added.end());
    }

    // ツリーを再構築
    build(maxBoidsPerUnit, 0);
}
void BoidTree::updatePositionBuffer()
{
    std::vector<Boid> boids = getBoids();
    positionBuffer.resize(boids.size() * 3);
    for (size_t i = 0; i < boids.size(); ++i)
    {
        positionBuffer[i * 3 + 0] = boids[i].position.x;
        positionBuffer[i * 3 + 1] = boids[i].position.y;
        positionBuffer[i * 3 + 2] = boids[i].position.z;
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
    std::vector<Boid> boids = getBoids();
    velocityBuffer.resize(boids.size() * 3);
    for (size_t i = 0; i < boids.size(); ++i)
    {
        velocityBuffer[i * 3 + 0] = boids[i].velocity.x;
        velocityBuffer[i * 3 + 1] = boids[i].velocity.y;
        velocityBuffer[i * 3 + 2] = boids[i].velocity.z;
    }
}

uintptr_t BoidTree::getVelocityBufferPtr()
{
    return reinterpret_cast<uintptr_t>(velocityBuffer.data());
}