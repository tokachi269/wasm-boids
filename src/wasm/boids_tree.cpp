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
BoidTree::BoidTree() : root(nullptr) {}

void BoidTree::build(int maxPerUnit, int level)
{
    maxBoidsPerUnit = maxPerUnit;

    if (!root)
    {
        root = new BoidUnit(); // 初回のみ新しいルートノードを作成
    }
    root->level = level;

    // 再帰的に木構造を構築
    std::vector<int> indices(root->positions.size());
    std::iota(indices.begin(), indices.end(), 0); // 0 から positions.size() - 1 までのインデックスを生成
    buildRecursive(root, indices, maxPerUnit, level);
}

void BoidTree::buildRecursive(BoidUnit *node, const std::vector<int> &ids, int maxPerUnit, int level)
{
    node->level = level;

    if ((int)ids.size() <= maxPerUnit)
    {
        // SoA形式で保存
        node->ids = ids;
        node->positions.clear();
        node->velocities.clear();
        node->accelerations.clear();

        for (int idx : ids)
        {
            node->positions.push_back( root->positions[idx]);
            node->velocities.push_back( root->velocities[idx]);
            node->accelerations.push_back( root->accelerations[idx]);
        }

        node->computeBoundingSphere();
        return;
    }

    // --- 最大分散軸の計算 ---
    float mean[3] = {0, 0, 0}, var[3] = {0, 0, 0};
    for (int idx : ids)
    {
        const auto &pos = root->positions[idx];
        mean[0] += pos.x;
        mean[1] += pos.y;
        mean[2] += pos.z;
    }
    for (int i = 0; i < 3; ++i)
        mean[i] /= ids.size();

    for (int idx : ids)
    {
        const auto &pos = root->positions[idx];
        var[0] += (pos.x - mean[0]) * (pos.x - mean[0]);
        var[1] += (pos.y - mean[1]) * (pos.y - mean[1]);
        var[2] += (pos.z - mean[2]) * (pos.z - mean[2]);
    }

    int axis = (var[1] > var[0]) ? 1 : 0;
    if (var[2] > var[axis])
        axis = 2;

    // --- ソート & 分割 ---
    std::vector<int> sortedIds = ids;
    std::sort(sortedIds.begin(), sortedIds.end(), [this, axis](int a, int b)
              {
        const auto &pa = root-> positions[a];
        const auto &pb =  root->positions[b];
        return (axis == 0) ? (pa.x < pb.x) :
               (axis == 1) ? (pa.y < pb.y) :
                             (pa.z < pb.z); });

    size_t mid = sortedIds.size() / 2;
    std::vector<int> leftIds(sortedIds.begin(), sortedIds.begin() + mid);
    std::vector<int> rightIds(sortedIds.begin() + mid, sortedIds.end());

    BoidUnit *left = new BoidUnit();
    BoidUnit *right = new BoidUnit();

    buildRecursive(left, leftIds, maxPerUnit, level + 1);
    buildRecursive(right, rightIds, maxPerUnit, level + 1);

    node->children = {left, right};
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

void BoidTree::update(float dt) {
    // 木構造全体を再帰的に更新
    if (root) {
        root->updateRecursive(dt);
    }

    frameCount++;

    // 一定フレームごとに葉ノードを再収集
    if (frameCount % 5 == 0) {
        leafCache.clear();
        collectLeaves(root, leafCache);
        splitIndex = 0;
        mergeIndex = 0;
    }

    // 一定フレームごとに木構造を再構築
    if (frameCount % 60 == 0) { // 再構築頻度を調整
        build(maxBoidsPerUnit, 0);
        updatePositionBuffer();
        updateVelocityBuffer();
        return;
    }

    // 分割と結合の処理
    if (!leafCache.empty()) {
        // 分割
        for (int i = 0; i < 5 && splitIndex < (int)leafCache.size(); ++i, ++splitIndex) {
            BoidUnit *u = leafCache[splitIndex];
            if (u && u->needsSplit(40.0f, 0.5f, maxBoidsPerUnit)) {
                u->splitInPlace(maxBoidsPerUnit);
                leafCache.clear();
                collectLeaves(root, leafCache);
                break;
            }
        }

        // 結合
        for (int i = 0; i < 5 && mergeIndex < (int)leafCache.size(); ++i, ++mergeIndex) {
            for (int j = mergeIndex + 1; j < (int)leafCache.size(); ++j) {
                BoidUnit *a = leafCache[mergeIndex];
                BoidUnit *b = leafCache[j];
                if (a && b && a->canMergeWith(*b)) {
                    BoidUnit *parent = findParent(root, b);
                    if (parent) {
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
    if (!root)
    {
        root = new BoidUnit(); // 初回のみ新しいルートノードを作成
    }

    // 配列を指定されたサイズにリサイズ
    root->positions.resize(count);
    root->velocities.resize(count);
    root->accelerations.resize(count);
    root->ids.resize(count);
    root->stresses.resize(count);
    root->speciesIds.resize(count);

    // ランダムな Boid を生成
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int i = 0; i < count; ++i)
    {
        root->positions[i] = glm::vec3(posDist(gen), posDist(gen), posDist(gen));
        root->velocities[i] = glm::vec3(velDist(gen), velDist(gen), velDist(gen));
        root->accelerations[i] = glm::vec3(0.0f);
        root->ids[i] = i;
        root->stresses[i] = 0.0f;
        root->speciesIds[i] = 0;
    }

    // 木構造を構築
    build(maxBoidsPerUnit, 0);
    printTree(root); // デバッグ用にツリーを出力
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
    positionBuffer.resize(root->positions.size() * 3);
    for (size_t i = 0; i < root->positions.size(); ++i)
    {
        positionBuffer[i * 3 + 0] = root->positions[i].x;
        positionBuffer[i * 3 + 1] = root->positions[i].y;
        positionBuffer[i * 3 + 2] = root->positions[i].z;
                        if (i == 3) // 最初のBoidのみログ出力
                {
                    std::cout << "Boid " << i << " position: " << glm::to_string(root->positions[i]) << " buffer" << std::endl;
                }
    }
        // デバッグログ
    std::cout << "Position Buffer Updated: " << positionBuffer.size() / 3 << " positions." << std::endl;

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
    velocityBuffer.resize(root->velocities.size() * 3);
    for (size_t i = 0; i < root->velocities.size(); ++i)
    {
        velocityBuffer[i * 3 + 0] = root->velocities[i].x;
        velocityBuffer[i * 3 + 1] = root->velocities[i].y;
        velocityBuffer[i * 3 + 2] = root->velocities[i].z;
    }
        // デバッグログ
    std::cout << "Velocity Buffer Updated: " << velocityBuffer.size() / 3 << " velocities." << std::endl;

}

uintptr_t BoidTree::getVelocityBufferPtr()
{
    return reinterpret_cast<uintptr_t>(velocityBuffer.data());
}