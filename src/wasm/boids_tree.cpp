#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"
#include <vector>
#include <algorithm>
#include <numeric>
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
    std::cout << globalSpeciesParams.torqueStrength << std::endl;
}
BoidTree::BoidTree()
    : root(nullptr), frameCount(0), splitIndex(0), mergeIndex(0), maxBoidsPerUnit(10)
{
}

// ---- ツリー可視化 ----
void printTree(const BoidUnit *node, int depth)
{
    if (!node)
        return;

    std::string indent(depth * 2, ' ');
    std::cout << indent
              << "Level: " << node->level
              << " | Boids: " << node->indices.size() // indices を参照
              << " | Children: " << node->children.size()
              << " | Center: (" << node->center.x << ", "
              << node->center.y << ", "
              << node->center.z << ")"
              << " | Radius: " << node->radius
              << '\n';

    for (const auto *child : node->children)
        printTree(child, depth + 1);
}

// ---- BoidTree::build ----
void BoidTree::build(int maxPerUnit, int level)
{
    if (!root)
    {
        root = new BoidUnit();
        root->buf = &buf; // 中央バッファを共有
    }

    root->level = level;

    // すべての Boid インデックスを作成
    std::vector<int> indices(buf.positions.size());
    std::iota(indices.begin(), indices.end(), 0);

    buildRecursive(root, indices, maxPerUnit, level);
}

void BoidTree::buildRecursive(BoidUnit *node, const std::vector<int> &indices, int maxPerUnit, int level)
{
    node->level = level;

    // 末端ノード（葉）の処理
    if ((int)indices.size() <= maxPerUnit)
    {
        node->indices = indices; // インデックスだけコピー
        node->computeBoundingSphere();
        return;
    }

    // 分割軸を決定するため平均と分散を計算
    float mean[3] = {0}, var[3] = {0};
    for (int i : indices)
    {
        const glm::vec3 &p = buf.positions[i];
        mean[0] += p.x;
        mean[1] += p.y;
        mean[2] += p.z;
    }
    for (int k = 0; k < 3; ++k)
        mean[k] /= indices.size();

    for (int i : indices)
    {
        const glm::vec3 &p = buf.positions[i];
        var[0] += (p.x - mean[0]) * (p.x - mean[0]);
        var[1] += (p.y - mean[1]) * (p.y - mean[1]);
        var[2] += (p.z - mean[2]) * (p.z - mean[2]);
    }
    int axis = (var[1] > var[0]) ? 1 : 0;
    if (var[2] > var[axis])
        axis = 2;

    // インデックスをソートして 2 つに分割
    std::vector<int> sorted = indices;
    std::sort(sorted.begin(), sorted.end(), [this, axis](int a, int b)
              {
        const glm::vec3& pa = buf.positions[a];
        const glm::vec3& pb = buf.positions[b];
        return (axis == 0) ? pa.x < pb.x
             : (axis == 1) ? pa.y < pb.y
                           : pa.z < pb.z; });
    std::size_t mid = sorted.size() / 2;
    std::vector<int> left(sorted.begin(), sorted.begin() + mid);
    std::vector<int> right(sorted.begin() + mid, sorted.end());

    // 子ノードを生成して中央バッファを共有
    auto *leftChild = new BoidUnit();
    auto *rightChild = new BoidUnit();
    leftChild->buf = node->buf;
    rightChild->buf = node->buf;
    node->children = {leftChild, rightChild};

    // 再帰処理
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
        splitIndex = 0;
        mergeIndex = 0;
    }

    // 一定フレームごとに木構造を再構築
    if (frameCount % 17 == 0)
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

// BoidTree::initializeBoids
void BoidTree::initializeBoids(int count, float posRange, float velRange)
{
    // 中央バッファを確保
    buf.reserveAll(count);
    buf.positions.resize(count);
    buf.velocities.resize(count);
    buf.accelerations.resize(count, glm::vec3(0.0f));
    buf.ids.resize(count);
    buf.stresses.resize(count, 0.0f);
    buf.speciesIds.resize(count, 0);
    buf.cohesionMemories.clear();
    for (int i = 0; i < count; ++i)
    {
        buf.cohesionMemories[i] = std::unordered_map<int, float>(); // 最大8件前提
    }

    // 乱数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    // 位置・速度を初期化
    for (int i = 0; i < count; ++i)
    {
        buf.positions[i] = glm::vec3(posDist(gen), posDist(gen), posDist(gen));
        buf.velocities[i] = glm::vec3(velDist(gen), velDist(gen), velDist(gen));
        buf.ids[i] = i;
    }

    // ルートノードが無ければ生成して中央バッファを共有
    if (!root)
        root = new BoidUnit();
    root->buf = &buf;

    // インデックス配列を作って木を構築
    std::vector<int> indices(count);
    std::iota(indices.begin(), indices.end(), 0);
    buildRecursive(root, indices, maxBoidsPerUnit, 0);
}

// BoidTree::setFlockSize
void BoidTree::setFlockSize(int newSize, float posRange, float velRange)
{
    int current = static_cast<int>(buf.positions.size());

    // 個体を減らす
    if (newSize < current)
    {
        buf.positions.resize(newSize);
        buf.velocities.resize(newSize);
        buf.accelerations.resize(newSize);
        buf.ids.resize(newSize);
        buf.stresses.resize(newSize);
        buf.speciesIds.resize(newSize);
        // cohesionMemories は erase で縮小
        for (int i = newSize; i < current; ++i)
        {
            buf.cohesionMemories.erase(i);
        }
    }
    // 個体を増やす
    else if (newSize > current)
    {
        int addN = newSize - current;
        buf.reserveAll(newSize);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> posDist(-posRange, posRange);
        std::uniform_real_distribution<float> velDist(-velRange, velRange);

        for (int k = 0; k < addN; ++k)
        {
            int i = current + k;
            buf.positions.push_back(glm::vec3(posDist(gen), posDist(gen), posDist(gen)));
            buf.velocities.push_back(glm::vec3(velDist(gen), velDist(gen), velDist(gen)));
            buf.accelerations.push_back(glm::vec3(0.0f));
            buf.ids.push_back(i);
            buf.stresses.push_back(0.0f);
            buf.speciesIds.push_back(0);
            buf.cohesionMemories[i] = std::unordered_map<int, float>();
        }
    }

    // ルートが中央バッファを指していることを保証
    if (!root)
        root = new BoidUnit();
    root->buf = &buf;

    // 再構築
    build(maxBoidsPerUnit, 0);
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

uintptr_t BoidTree::getPositionsPtr()
{
    return reinterpret_cast<uintptr_t>(&buf.positions[0]);
}

uintptr_t BoidTree::getVelocitiesPtr()
{
    return reinterpret_cast<uintptr_t>(&buf.velocities[0]);
}

int BoidTree::getBoidCount() const
{
    return static_cast<int>(buf.positions.size());
}