#define GLM_ENABLE_EXPERIMENTAL
#include "boid_unit.h"
#include "boids_tree.h"
#include <algorithm>
#include <queue>
#include <iostream>
#include <random>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/random.hpp>
#include <random>

bool BoidUnit::isBoidUnit() const { return children.empty(); }

inline glm::vec3 approxRotate(const glm::vec3 &v, const glm::vec3 &axis, float angle)
{
    // 小角度近似: sinθ ≈ θ, cosθ ≈ 1
    return v + angle * glm::cross(axis, v);
}

/**
 * ユニット内の Boid または子ノードを基にバウンディングスフィアを計算する。
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - Boid の位置を基に中心と半径を計算。
 * - **中間ノードの場合**:
 *   - 子ノードの中心と半径を基に親ノードの中心と半径を計算。
 *
 * 使用例:
 * - 階層構造内でユニットの境界を計算する際に使用。
 */
void BoidUnit::computeBoundingSphere()
{
    if (isBoidUnit())
    {
        if (indices.empty())
            return;

        // 中心を計算
        center = glm::vec3(0.0f);
        for (int gIdx : indices)
            center += buf->positions[gIdx];
        center /= static_cast<float>(indices.size());

        // 平均距離と分散を計算
        float sum = 0.0f, sum2 = 0.0f;
        for (int gIdx : indices)
        {
            float d = glm::distance(center, buf->positions[gIdx]);
            sum += d;
            sum2 += d * d;
        }
        float mean = sum / static_cast<float>(indices.size());
        float var = sum2 / static_cast<float>(indices.size()) - mean * mean;
        float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

        // 平均 + α × 標準偏差（α = 1.0）で半径を決定
        radius = mean + 1.0f * stddev;
    }
    else
    {
        if (children.empty())
            return;

        // 子ノードの中心を計算
        center = glm::vec3(0.0f);
        for (const auto &child : children)
            center += child->center;
        center /= static_cast<float>(children.size());

        // 子ノード中心までの平均距離 + 子ノード半径
        float sum = 0.0f, sum2 = 0.0f;
        for (const auto &child : children)
        {
            float d = glm::distance(center, child->center) + child->radius;
            sum += d;
            sum2 += d * d;
        }
        float mean = sum / static_cast<float>(children.size());
        float var = sum2 / static_cast<float>(children.size()) - mean * mean;
        float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

        radius = mean + 1.0f * stddev;
    }
}

/**
 * 他のユニットとの相互作用を計算し、加速度に影響を加える。
 *
 * @param other 他の BoidUnit へのポインタ
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - Boid 間の詳細な相互作用を計算し、分離、整列、凝集ルールを適用。
 * - **片方が中間ノードの場合**:
 *   - 再帰的に子ノードを処理。
 * - **両方が中間ノードの場合**:
 *   - 代表値（中心、平均速度）を用いて近似計算を行う。
 *
 * 使用例:
 * - 階層構造内で異なるユニット間の影響を計算する際に使用。
 */
void BoidUnit::applyInterUnitInfluence(BoidUnit *other, float dt)
{
    if (!indices.empty() && !other->indices.empty())
    {

        // 葉ノード同士
        for (int idxA : indices)
        {

            glm::vec3 sumVel = glm::vec3(0.0f);
            glm::vec3 sumPos = glm::vec3(0.0f);
            glm::vec3 sep = glm::vec3(0.0f);
            int cnt = 0;

            for (int idxB : other->indices)
            {
                glm::vec3 diff = buf->positions[idxA] - other->buf->positions[idxB];
                float d2 = glm::dot(diff, diff);

                if (d2 < 2500.0f && d2 > 1e-4f)
                {
                    float d = std::sqrt(d2);
                    float w = std::max(0.0f, 1.0f - (d / 40.0f));

                    sumVel += other->buf->velocities[idxB] * w;
                    sumPos += other->buf->positions[idxB] * w;
                    sep += (diff / (d2 + 1.0f)) * w;
                    ++cnt;
                }
            }

            if (cnt > 0)
            {

                // 整列・凝集・分離
                buf->accelerations[idxA] +=
                    (sumVel / float(cnt) - buf->velocities[idxA]) * globalSpeciesParams.alignment;
                buf->accelerations[idxA] +=
                    (sumPos / float(cnt) - buf->positions[idxA]) * (globalSpeciesParams.cohesion * 0.5f);
                buf->accelerations[idxA] +=
                    sep * (globalSpeciesParams.separation * 0.5f);

                // ── 回転トルク（復活部分） ──
                glm::vec3 fwd = glm::normalize(buf->velocities[idxA]);
                glm::vec3 tgt = glm::normalize(sumVel / float(cnt));
                float dot = glm::clamp(glm::dot(fwd, tgt), -1.0f, 1.0f);
                float ang = acosf(dot);

                if (ang > 1e-4f)
                {
                    glm::vec3 axis = glm::cross(fwd, tgt);
                    float len = glm::length(axis);
                    if (len > 1e-4f)
                    {
                        axis /= len;

                        float rot = std::min(ang, globalSpeciesParams.torqueStrength * dt);
                        rot = std::min(rot, globalSpeciesParams.maxTurnAngle);

                        glm::vec3 newDir = approxRotate(fwd, axis, rot);
                        buf->velocities[idxA] = newDir * glm::length(buf->velocities[idxA]);

                        // 加速度にもトルク分を加算（任意: 回転のノイズを弱めたい場合は外して良い）
                        buf->accelerations[idxA] += axis * ang * globalSpeciesParams.torqueStrength;
                    }
                }
            }
        }
    }
    else if (!indices.empty() && other->indices.empty())
    {

        // this が葉, other が中間
        for (auto *c : other->children)
            applyInterUnitInfluence(c, dt);
    }
    else if (indices.empty() && !other->indices.empty())
    {

        // this が中間, other が葉
        for (auto *c : children)
            c->applyInterUnitInfluence(other, dt);
    }
    else
    {

        // 中間ノード同士（代表値近似）
        float d2 = glm::distance2(center, other->center);
        if (d2 < 1600.0f && d2 > 0.01f)
        {

            float d = glm::sqrt(d2);
            float scale = 1.0f / (d2 + 1.0f);

            glm::vec3 diff = center - other->center;
            glm::vec3 separ = diff / d2 * globalSpeciesParams.separation;
            glm::vec3 align = (other->averageVelocity - averageVelocity) * globalSpeciesParams.alignment;
            glm::vec3 cohes = (other->center - center) * globalSpeciesParams.cohesion;

            glm::vec3 combined = (align + cohes + separ) * scale;
            for (int idx : indices)
                buf->accelerations[idx] += combined;

            for (int idx : other->indices)
                other->buf->accelerations[idx] -= combined;
        }
    }
}

/**
 * 再帰的にユニット内の Boid の動きを更新する。
 *
 * @param dt 時間ステップ
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - 各 Boid の加速度を初期化。
 *   - 分離、整列、凝集ルールを適用して加速度を計算。
 *   - 最大速度、最小速度、最大旋回角を考慮して速度を更新。
 *   - Boid の位置を更新。
 * - **中間ノードの場合**:
 *   - 再帰的に子ノードを処理。
 *   - 子ノードの結果（中心、平均速度など）を親ノードに伝搬。
 *
 * 使用例:
 * - 階層構造内で各ユニットの Boid の動きを更新する際に使用。
 */
void BoidUnit::updateRecursive(float dt)
{
    frameCount++;
    std::stack<BoidUnit *> stack;
    stack.push(this);

    // 第一段階: acceleration をすべて計算
    while (!stack.empty())
    {
        BoidUnit *current = stack.top();
        stack.pop();

        if (current->isBoidUnit())
        {
            current->computeBoidInteraction(dt);
        }
        else
        {
            for (auto &child : current->children)
                stack.push(child);

            // 子ノード同士の影響
            for (size_t a = 0; a < current->children.size(); ++a)
                for (size_t b = a + 1; b < current->children.size(); ++b)
                    current->children[a]->applyInterUnitInfluence(current->children[b]);

            current->computeBoundingSphere();
        }
    }

    // 第二段階: acceleration 適用 → velocity / position 更新
    stack.push(this);
    while (!stack.empty())
    {
        BoidUnit *current = stack.top();
        stack.pop();

        if (current->isBoidUnit())
        {
            for (size_t i = 0; i < current->indices.size(); ++i)
            {
                int gIdx = current->indices[i];

                glm::vec3 velocity = current->buf->velocities[gIdx];
                glm::vec3 acceleration = current->buf->accelerations[gIdx];
                glm::vec3 desiredVelocity = velocity + acceleration * dt;

                glm::vec3 oldDir = glm::normalize(velocity);
                glm::vec3 newDir = glm::normalize(desiredVelocity);
                float speed = glm::length(desiredVelocity);

                float dotProduct = glm::dot(oldDir, newDir);
                float angle = acosf(glm::clamp(dotProduct, -1.0f, 1.0f));

                if (angle > globalSpeciesParams.maxTurnAngle)
                {
                    glm::vec3 axis = glm::cross(oldDir, newDir);
                    float axisLength2 = glm::length2(axis);
                    if (axisLength2 > 1e-8f)
                    {
                        axis /= glm::sqrt(axisLength2); // 正規化
                        float rot = glm::min(angle, globalSpeciesParams.maxTurnAngle * dt);
                        newDir = approxRotate(oldDir, axis, rot);
                    }
                }

                float tilt = newDir.y;
                if (fabsf(tilt) > 1e-4f)
                {
                    glm::vec3 flatDir = glm::normalize(glm::vec3(newDir.x, 0, newDir.z));
                    glm::vec3 axis = glm::cross(newDir, flatDir);
                    float flatAngle = acosf(glm::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f));
                    float axisLength2 = glm::length2(axis);
                    if (flatAngle > 1e-4f && axisLength2 > 1e-8f)
                    {
                        axis /= glm::sqrt(axisLength2); // 正規化
                        float rot = glm::min(flatAngle, globalSpeciesParams.horizontalTorque * dt);
                        newDir = approxRotate(newDir, axis, rot);
                    }
                }

                float finalSpeed = glm::clamp(speed, globalSpeciesParams.minSpeed, globalSpeciesParams.maxSpeed);

                current->buf->velocities[gIdx] = newDir * finalSpeed;
                current->buf->positions[gIdx] += current->buf->velocities[gIdx] * dt;
                current->buf->accelerations[gIdx] = glm::vec3(0.0f); // 加速度リセット
            }
        }
        else
        {
            for (auto &child : current->children)
                stack.push(child);
        }
    }
}

void BoidUnit::computeBoidInteraction(float dt)
{
    glm::vec3 separation;
    glm::vec3 alignment;
    glm::vec3 cohesion;

    int count = 0;
    int memCount = 0;
    int gIdx = 0;
    glm::vec3 pos;
    glm::vec3 vel;

    for (size_t index = 0; index < indices.size(); ++index)
    {
        separation = glm::vec3(0.00001f);
        alignment = glm::vec3(0.00001f);
        cohesion = glm::vec3(0.00001f);

        count = 0;
        gIdx = indices[index];
        pos = buf->positions[gIdx];
        vel = buf->velocities[gIdx];

        // 1. 時間を更新し、無効化フラグを設定
        // 1. 時間更新＆無効化
        for (size_t i = 0; i < MAX_BOIDS; ++i)
        {
            if (cohesionMemories[i] > 0.0f)
            {
                cohesionMemories[i] += dt;
                if (cohesionMemories[i] > globalSpeciesParams.tau)
                {
                    cohesionMemories[i] = -1.0f;
                    activeNeighbors.reset(i);
                }
            }
        }

        // 2. 新規近傍追加（空きスロットがあるとき）
        if (activeNeighbors.count() < static_cast<size_t>(globalSpeciesParams.maxNeighbors))
        {
            for (size_t i = 0; i < indices.size(); ++i)
            {
                if (i == index)
                    continue;

                int gNeighbor = indices[i];
                glm::vec3 diff = pos - buf->positions[gNeighbor];
                float distSq = glm::dot(diff, diff);

                if (distSq < globalSpeciesParams.cohesionRange * globalSpeciesParams.cohesionRange)
                {
                    if (!activeNeighbors.test(i))
                    {
                        cohesionMemories[i] = dt;
                        activeNeighbors.set(i);
                    }
                }
            }
        }

        // 3. 影響計算
        for (size_t i = 0; i < MAX_BOIDS; ++i)
        {
            if (cohesionMemories[i] > 0.0f && activeNeighbors.test(i))
            {
                int gNeighbor = indices[i];
                glm::vec3 diff = pos - buf->positions[gNeighbor];
                float distSq = glm::dot(diff, diff);

                if (distSq > 0.0001f)
                {
                    float dist = sqrtf(distSq);
                    float weight = 1.0f - (dist / globalSpeciesParams.cohesionRange);
                    weight = glm::clamp(weight, 0.0f, 1.0f);

                    separation += diff * (weight * globalSpeciesParams.separation);
                    cohesion += buf->positions[gNeighbor] * weight;
                    alignment += buf->velocities[gNeighbor];
                    count++;
                }
            }
        }

        memCount = cohesionMemories.size();
        // 3. cohesionMemories に基づいて影響を反映
        if (memCount > 0)
        {
            glm::vec3 totalCohesion = cohesion / static_cast<float>(memCount) - pos;
            if (glm::length2(totalCohesion) > 0.0f)
                totalCohesion = glm::normalize(totalCohesion) * globalSpeciesParams.cohesion * 1.2f;

            if (count > 0)
            {
                separation = glm::normalize(separation) * globalSpeciesParams.separation * 0.8f;
                alignment = (alignment / static_cast<float>(count) - vel) * globalSpeciesParams.alignment;

                // 回転トルクの適用
                glm::vec3 fwd = glm::normalize(vel);
                glm::vec3 tgt = glm::normalize(alignment);
                float dot = glm::clamp(glm::dot(fwd, tgt), -1.0f, 1.0f);
                float angle = acosf(dot);

                if (angle > 1e-4f)
                {
                    glm::vec3 axis = glm::cross(fwd, tgt);
                    float len = glm::length(axis);
                    if (len > 1e-4f)
                    {
                        axis /= len;

                        float rot = std::min(angle, globalSpeciesParams.torqueStrength * dt);
                        rot = std::min(rot, globalSpeciesParams.maxTurnAngle);

                        glm::vec3 newDir = approxRotate(fwd, axis, rot);
                        vel = newDir * glm::length(vel);

                        // 加速度にもトルク分を加算
                        buf->accelerations[gIdx] += axis * angle * globalSpeciesParams.torqueStrength;
                    }
                }
            }

            buf->accelerations[gIdx] += separation + alignment + totalCohesion;
        }
    }
}

// 分割が必要か判定 (indices + 中央バッファ版)
bool BoidUnit::needsSplit(float splitRadius, float directionVarThresh, int maxBoids) const
{
    if (static_cast<int>(indices.size()) > maxBoids)
        return true;
    if (radius > splitRadius)
        return true;

    // 方向のバラつき判定
    if (indices.size() > 1)
    {
        glm::vec3 avg = glm::vec3(0.0f);
        for (int gIdx : indices)
            avg += glm::normalize(buf->velocities[gIdx]);
        avg /= static_cast<float>(indices.size());

        float var = 0.0f;
        for (int gIdx : indices)
            var += glm::length(glm::normalize(buf->velocities[gIdx]) - avg);
        var /= static_cast<float>(indices.size());

        if (var > directionVarThresh)
            return true;
    }
    return false;
}

// 最大分散軸で分割 → 任意分割 (indices 版)
std::vector<BoidUnit *> BoidUnit::split(int numSplits)
{
    if (numSplits < 2)
        numSplits = 2;
    if (static_cast<int>(indices.size()) < numSplits)
        numSplits = static_cast<int>(indices.size());

    // 最大分散軸を求める
    int axis = 0;
    float maxVar = 0.0f;
    for (int ax = 0; ax < 3; ++ax)
    {
        float mean = 0.0f, var = 0.0f;
        for (int gIdx : indices)
            mean += (ax == 0   ? buf->positions[gIdx].x
                     : ax == 1 ? buf->positions[gIdx].y
                               : buf->positions[gIdx].z);
        mean /= static_cast<float>(indices.size());

        for (int gIdx : indices)
        {
            float v = (ax == 0   ? buf->positions[gIdx].x
                       : ax == 1 ? buf->positions[gIdx].y
                                 : buf->positions[gIdx].z) -
                      mean;
            var += v * v;
        }
        if (var > maxVar)
        {
            maxVar = var;
            axis = ax;
        }
    }

    // min / max を取り等間隔で分ける
    float minVal = std::numeric_limits<float>::max();
    float maxVal = -std::numeric_limits<float>::max();
    for (int gIdx : indices)
    {
        float v = (axis == 0   ? buf->positions[gIdx].x
                   : axis == 1 ? buf->positions[gIdx].y
                               : buf->positions[gIdx].z);
        minVal = std::min(minVal, v);
        maxVal = std::max(maxVal, v);
    }

    float interval = (maxVal - minVal) / numSplits;
    std::vector<std::vector<int>> groups(numSplits);
    for (int gIdx : indices)
    {
        float v = (axis == 0   ? buf->positions[gIdx].x
                   : axis == 1 ? buf->positions[gIdx].y
                               : buf->positions[gIdx].z);
        int idx = std::min(numSplits - 1, int((v - minVal) / interval));
        groups[idx].push_back(gIdx);
    }

    // 子 BoidUnit を生成
    std::vector<BoidUnit *> result;
    for (const auto &g : groups)
    {
        if (g.empty())
            continue;
        BoidUnit *u = new BoidUnit();
        u->buf = buf;   // 中央バッファ共有
        u->indices = g; // インデックスだけ保持
        u->level = level + 1;
        u->computeBoundingSphere();
        result.push_back(u);
    }
    return result;
}

// 現在のユニットを分割し子ノードとして配置
void BoidUnit::splitInPlace(int maxBoids)
{
    if (!needsSplit(80.0f, 0.5f, maxBoids))
        return;

    auto splits = splitByClustering(4);

    // 自分は中間ノードになるので indices を空に
    indices.clear();

    // 子ノードを登録
    children = std::move(splits);

    // バウンディングスフィアを再計算
    computeBoundingSphere();
}
// k-means 風クラスタリングで indices をグループ化
std::vector<BoidUnit *> BoidUnit::splitByClustering(int numClusters)
{
    if ((int)indices.size() < numClusters)
        numClusters = static_cast<int>(indices.size());
    if (numClusters < 2)
        numClusters = 2;

    // 初期中心をランダム（ここでは先頭から）に選択
    std::vector<glm::vec3> centers;
    for (int k = 0; k < numClusters; ++k)
        centers.push_back(buf->positions[indices[k]]);

    std::vector<int> assign(indices.size(), 0);

    // 反復回数は少なめに固定
    for (int iter = 0; iter < 5; ++iter)
    {
        // 割り当て
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int gI = indices[i];
            float best = std::numeric_limits<float>::max();
            int bestK = 0;
            for (int k = 0; k < numClusters; ++k)
            {
                float d = glm::distance(buf->positions[gI], centers[k]);
                if (d < best)
                {
                    best = d;
                    bestK = k;
                }
            }
            assign[i] = bestK;
        }

        // 中心を再計算
        std::vector<glm::vec3> newCenters(numClusters, glm::vec3(0.0f));
        std::vector<int> counts(numClusters, 0);
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int gI = indices[i];
            newCenters[assign[i]] += buf->positions[gI];
            counts[assign[i]]++;
        }
        for (int k = 0; k < numClusters; ++k)
        {
            if (counts[k] > 0)
                newCenters[k] /= static_cast<float>(counts[k]);
            else
                newCenters[k] = centers[k];
        }
        centers.swap(newCenters);
    }

    // グループごとに global index をまとめる
    std::vector<std::vector<int>> groups(numClusters);
    for (size_t i = 0; i < indices.size(); ++i)
        groups[assign[i]].push_back(indices[i]);

    // 各グループから BoidUnit を生成
    std::vector<BoidUnit *> result;
    for (const auto &g : groups)
    {
        if (g.empty())
            continue;
        auto *u = new BoidUnit();
        u->buf = buf;   // 中央バッファを共有
        u->indices = g; // インデックスだけ保持
        u->level = level + 1;
        u->computeBoundingSphere();
        result.push_back(u);
    }
    return result;
}

/**
 * 指定されたユニット内のすべての Boid に反発力を加える。
 *
 * @param unit 対象の BoidUnit
 * @param repulsion 反発力ベクトル
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - Boid の位置に基づいて反発力を計算し、加速度に加算。
 * - **中間ノードの場合**:
 *   - 再帰的に子ノードを処理。
 *
 * 使用例:
 * - Boid 間の衝突回避やユニット間の分離を実現する際に使用。
 */
bool BoidUnit::canMergeWith(const BoidUnit &other,
                            float mergeDist, float velThresh,
                            float maxRadius, int maxBoids) const
{
    // 中心間距離
    if (glm::distance(center, other.center) > mergeDist)
        return false;

    // 平均速度差
    if (glm::length(averageVelocity - other.averageVelocity) > velThresh)
        return false;

    // Boid 数上限
    if (static_cast<int>(indices.size() + other.indices.size()) > maxBoids)
        return false;

    // 結合後の中心
    glm::vec3 newCenter =
        (center * static_cast<float>(indices.size()) +
         other.center * static_cast<float>(other.indices.size())) /
        static_cast<float>(indices.size() + other.indices.size());

    // 結合後の半径
    float newRadius = 0.0f;
    for (int gIdx : indices)
        newRadius = std::max(newRadius, glm::distance(newCenter, buf->positions[gIdx]));
    for (int gIdx : other.indices)
        newRadius = std::max(newRadius, glm::distance(newCenter, buf->positions[gIdx]));

    return newRadius <= maxRadius;
}

// 他ユニットを結合（値渡し版）
void BoidUnit::mergeWith(const BoidUnit &other)
{
    indices.insert(indices.end(), other.indices.begin(), other.indices.end());
    computeBoundingSphere();
}

// 他ユニットを結合（ポインタ／親ノード付き版）
void BoidUnit::mergeWith(BoidUnit *other, BoidUnit *parent)
{
    indices.insert(indices.end(), other->indices.begin(), other->indices.end());
    children.clear(); // 自ノードを葉に戻す
    computeBoundingSphere();

    if (parent)
    {
        auto it = std::find(parent->children.begin(), parent->children.end(), other);
        if (it != parent->children.end())
            parent->children.erase(it);
    }
    delete other;
}

// 兄弟ノード配下の全 Boid に反発を適用
void BoidUnit::addRepulsionToAllBoids(BoidUnit *unit, const glm::vec3 &repulsion)
{
    if (unit->isBoidUnit())
    {
        for (int gIdx : unit->indices)
        {
            float d = glm::length(unit->buf->positions[gIdx] - unit->center);
            float w = 0.5f + 0.5f * (d / (unit->radius + 1e-5f)); // 端ほど 1.0、中心 0.5
            unit->buf->accelerations[gIdx] += repulsion * w;
        }
    }
    else
    {
        for (auto *c : unit->children)
            addRepulsionToAllBoids(c, repulsion);
    }
}

int BoidUnit::getMaxID() const
{
    int maxID = -1; // 初期値を -1 に設定（ID が負の値になることはないと仮定）
    for (int idx : indices)
    {
        if (buf->ids[idx] > maxID)
        {
            maxID = buf->ids[idx];
        }
    }

    // 子ノードがある場合、再帰的に最大 ID を取得
    for (const auto *child : children)
    {
        if (child)
        {
            maxID = std::max(maxID, child->getMaxID());
        }
    }

    return maxID;
}