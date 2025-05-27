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
        if (positions.empty())
            return;

        // 中心を計算
        center = glm::vec3(0.0f);
        for (const auto &pos : positions)
            center += pos;
        center /= static_cast<float>(positions.size());

        // 平均距離と分散を計算
        float sum = 0.0f, sum2 = 0.0f;
        for (const auto &pos : positions)
        {
            float d = glm::distance(center, pos);
            sum += d;
            sum2 += d * d;
        }
        float mean = sum / static_cast<float>(positions.size());
        float var = sum2 / static_cast<float>(positions.size()) - mean * mean;
        float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

        // 平均 + α × 標準偏差（例: α = 1.5）で半径を決定
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

        // 子ノード中心までの平均距離 + 子ノードの半径の平均 + α × 標準偏差
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
void BoidUnit::applyInterUnitInfluence(BoidUnit *other)
{
    if (!positions.empty() && !other->positions.empty())
    {
        // 葉ノード同士はBoidごとに詳細計算
        for (size_t i = 0; i < positions.size(); ++i)
        {
            glm::vec3 sumVelocity = glm::vec3(0.0f);
            glm::vec3 sumPosition = glm::vec3(0.0f);
            glm::vec3 separation = glm::vec3(0.0f);
            int count = 0;

            for (size_t j = 0; j < other->positions.size(); ++j)
            {
                glm::vec3 diff = positions[i] - other->positions[j];
                float distSq = glm::dot(diff, diff);

                if (distSq < 2500.0f && distSq > 0.0001f) // 50.0f^2 = 2500.0f
                {
                    float dist = std::sqrt(distSq); // 必要な場合のみ平方根を計算
                    float influence = std::max(0.0f, 1.0f - (dist / 40.0f));

                    sumVelocity += other->velocities[j] * influence;
                    sumPosition += other->positions[j] * influence;
                    separation += (diff / (dist * dist + 1.0f)) * influence;
                    count++;
                }
            }

            if (count > 0)
            {
                accelerations[i] += ((sumVelocity / static_cast<float>(count) - velocities[i]) * globalSpeciesParams.alignment);
                accelerations[i] += ((sumPosition / static_cast<float>(count) - positions[i]) * (globalSpeciesParams.cohesion * 0.5f)); // 凝集の影響を50%に
                accelerations[i] += (separation * (globalSpeciesParams.separation * 0.5f));                                             // 分離の影響を50%に
            }
        }
    }
    else if (!positions.empty() && other->positions.empty())
    {
        // 片方が葉ノード、片方が中間ノード
        for (auto *c : other->children)
        {
            applyInterUnitInfluence(c);
        }
    }
    else if (positions.empty() && !other->positions.empty())
    {
        // 片方が中間ノード、もう片方が葉ノード
        for (auto *c : children)
        {
            c->applyInterUnitInfluence(other);
        }
    }
    else
    {
        // 両方とも中間ノード → 代表値で近似計算
        float distSq = glm::distance2(center, other->center);

        if (distSq < 1600.0f && distSq > 0.01f)
        {
            float dist = glm::sqrt(distSq);

            float influence = 1.0f / (dist * dist + 1.0f); // 距離減衰
            float scale = influence;

            glm::vec3 diff = center - other->center;
            glm::vec3 separation = diff / (dist * dist) * globalSpeciesParams.separation;
            glm::vec3 align = (other->averageVelocity - averageVelocity) * globalSpeciesParams.alignment;
            glm::vec3 cohes = (other->center - center) * globalSpeciesParams.cohesion;

            // このノード配下の全Boidのインデックスを収集
            std::vector<int> allBoidIndices;
            std::queue<BoidUnit *> q;
            q.push(this);
            while (!q.empty())
            {
                BoidUnit *u = q.front();
                q.pop();
                if (u->isBoidUnit())
                {
                    allBoidIndices.insert(allBoidIndices.end(), u->ids.begin(), u->ids.end());
                }
                else
                {
                    for (BoidUnit *c : u->children)
                    {
                        q.push(c);
                    }
                }
            }

            // 相手ノード配下の全Boidのインデックスを収集
            std::vector<int> otherBoidIndices;
            q.push(other);
            while (!q.empty())
            {
                BoidUnit *u = q.front();
                q.pop();
                if (u->isBoidUnit())
                {
                    otherBoidIndices.insert(otherBoidIndices.end(), u->ids.begin(), u->ids.end());
                }
                else
                {
                    for (BoidUnit *c : u->children)
                    {
                        q.push(c);
                    }
                }
            }

            // 影響を加算
            for (size_t i = 0; i < positions.size(); ++i)
            {
                glm::vec3 fwd = glm::normalize(velocities[i]);
                glm::vec3 target = glm::normalize(other->center - center);
                float dot = glm::clamp(glm::dot(fwd, target), -1.0f, 1.0f);
                float angle = acos(dot);
                glm::vec3 axis = glm::cross(fwd, target);
                float torqueStrength = globalSpeciesParams.torqueStrength; // 必要に応じて調整

                if (angle > 1e-4f)
                {
                    float axisLen = glm::length(axis);
                    if (axisLen > 1e-4f)
                    {
                        axis = axis / axisLen;

                        // 距離減衰とパラメータで回転量を調整
                        float rotateAngle = std::min(angle, torqueStrength * scale);
                        rotateAngle = std::min(rotateAngle, globalSpeciesParams.maxTurnAngle);

                        // 回転トルクを適用
                        glm::vec3 newDir = glm::rotate(fwd, rotateAngle, axis);
                        velocities[i] = newDir * glm::length(velocities[i]);
                    }
                }

                // 回転トルクに基づく加速度の調整
                glm::vec3 torque = axis * angle * torqueStrength;
                accelerations[i] += torque;
            }

            for (size_t j = 0; j < other->positions.size(); ++j)
            {
                glm::vec3 fwd = glm::normalize(other->velocities[j]);
                glm::vec3 target = glm::normalize(center - other->center);
                float dot = glm::clamp(glm::dot(fwd, target), -1.0f, 1.0f);
                float angle = acos(dot);
                if (angle > 1e-4f)
                {
                    glm::vec3 axis = glm::cross(fwd, target);
                    float axisLen = glm::length(axis);
                    if (axisLen > 1e-4f)
                    {
                        axis = axis / axisLen;
                        float rotateAngle = std::min(angle, globalSpeciesParams.torqueStrength * scale);
                        rotateAngle = std::min(rotateAngle, globalSpeciesParams.maxTurnAngle);
                        glm::vec3 newDir = glm::rotate(fwd, rotateAngle, axis);
                        other->velocities[j] = newDir * glm::length(other->velocities[j]);
                    }
                }

                // 加速度の調整
                other->accelerations[j] -= align * scale;
                other->accelerations[j] -= cohes * scale;
                other->accelerations[j] -= separation * scale;
            }
        }
    }
}
inline glm::vec3 approxRotate(const glm::vec3 &v, const glm::vec3 &axis, float angle)
{
    // 小角度近似: sinθ ≈ θ, cosθ ≈ 1
    return v + angle * glm::cross(axis, v);
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
            for (size_t i = 0; i < current->positions.size(); ++i)
            {
                current->computeBoidInteraction(i, dt);
            }
        }
        else
        {
            for (auto &child : current->children)
                stack.push(child);

            for (auto &other : current->children)
                if (current != other)
                    current->applyInterUnitInfluence(other);

            current->computeBoundingSphere();
        }
    }

    // 第二段階: acceleration 適用 → velocity/position 更新
    stack.push(this);
    while (!stack.empty())
    {
        BoidUnit *current = stack.top();
        stack.pop();

        if (current->isBoidUnit())
        {
            for (size_t i = 0; i < current->positions.size(); ++i)
            {
                glm::vec3 desiredVelocity = current->velocities[i] + current->accelerations[i] * dt;
                glm::vec3 oldDir = glm::normalize(current->velocities[i]);
                glm::vec3 newDir = glm::normalize(desiredVelocity);
                float speed = glm::length(desiredVelocity);

                float angle = acosf(glm::clamp(glm::dot(oldDir, newDir), -1.0f, 1.0f));
                if (angle > globalSpeciesParams.maxTurnAngle)
                {
                    glm::vec3 axis = glm::cross(oldDir, newDir);
                    if (glm::length2(axis) > 1e-8f)
                    {
                        axis = glm::normalize(axis);
                        float rotateAngle = glm::min(angle, globalSpeciesParams.maxTurnAngle * dt);
                        newDir = approxRotate(oldDir, axis, rotateAngle); // glm::rotate を approxRotate に置き換え
                    }
                }

                float tilt = newDir.y;
                if (fabsf(tilt) > 1e-4f)
                {
                    glm::vec3 flatDir = glm::normalize(glm::vec3(newDir.x, 0, newDir.z));
                    glm::vec3 axis = glm::cross(newDir, flatDir);
                    float flatAngle = acosf(glm::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f));
                    if (flatAngle > 1e-4f && glm::length2(axis) > 1e-8f)
                    {
                        axis = glm::normalize(axis);
                        float rotateAngle = glm::min(flatAngle, globalSpeciesParams.horizontalTorque * dt);
                        newDir = approxRotate(newDir, axis, rotateAngle); // glm::rotate を approxRotate に置き換え
                    }
                }

                float finalSpeed = glm::clamp(speed, globalSpeciesParams.minSpeed, globalSpeciesParams.maxSpeed);
                current->velocities[i] = newDir * finalSpeed;
                current->positions[i] += current->velocities[i] * dt;
                current->accelerations[i] = glm::vec3(0.0f); // 加速度をリセット
            }
        }
        else
        {
            for (auto &child : current->children)
                stack.push(child);
        }
    }
}

void BoidUnit::computeBoidInteraction(size_t index, float dt)
{
    std::cout << "Computing interaction for Boid at index: " << index << std::endl;
    glm::vec3 separation = glm::vec3(0.0f);
    glm::vec3 alignment = glm::vec3(0.0f);
    glm::vec3 cohesion = glm::vec3(0.0f);
    glm::vec3 memCohesion = glm::vec3(0.0f);

    int count = 0;
    int memCount = 0;

    glm::vec3 pos = positions[index];
    glm::vec3 vel = velocities[index];

    // 近傍Boidとの相互作用を計算
    std::unordered_set<int> visibleIds;
    visibleIds.reserve(positions.size());

    for (size_t j = 0; j < positions.size(); ++j)
    {
        if (j == index)
            continue;

        glm::vec3 diff = pos - positions[j];
        float distSq = glm::dot(diff, diff);

        if (distSq < 2500.0f && distSq > 0.0001f)
        {
            float dist = sqrtf(distSq);

            // 分離
            float separationWeight = glm::clamp(1.0f - (dist / 50.0f), 0.0f, 1.0f);
            separation += diff * (separationWeight * globalSpeciesParams.separation);

            // 凝集
            float cohesionWeight = glm::clamp(dist / 50.0f, 0.0f, 1.0f);
            cohesion += positions[j] * cohesionWeight;

            // 整列
            alignment += velocities[j];
            count++;

            // 視界内のBoidを記録
            visibleIds.insert(ids[j]);
        }
    }

    // cohesionMemory の更新を隔フレームで実行
    if (frameCount % 5 == 0)
    {
        for (auto it = cohesionMemory[ids[index]].begin(); it != cohesionMemory[ids[index]].end();)
        {
            if (visibleIds.find(it->first) == visibleIds.end())
            {
                it->second += dt;
                if (it->second > globalSpeciesParams.tau)
                {
                    it = cohesionMemory[ids[index]].erase(it);
                    continue;
                }
            }
            ++it;
        }
    }

    // cohesionMemory のサイズを制限 (LRU風)
    const size_t maxCohesionMemorySize = 100;
    if (cohesionMemory[ids[index]].size() > maxCohesionMemorySize)
    {
        auto oldest = std::min_element(
            cohesionMemory[ids[index]].begin(), cohesionMemory[ids[index]].end(),
            [](const auto &a, const auto &b)
            { return a.second > b.second; });
        if (oldest != cohesionMemory[ids[index]].end())
        {
            cohesionMemory[ids[index]].erase(oldest);
        }
    }

    // cohesionMemory を使用した凝集計算
    for (const auto &mem : cohesionMemory[ids[index]])
    {
        for (size_t j = 0; j < ids.size(); ++j)
        {
            if (ids[j] == mem.first)
            {
                memCohesion += positions[j];
                memCount++;
                break;
            }
        }
    }

    if (count + memCount > 0)
    {
        glm::vec3 totalCohesion = cohesion + memCohesion;
        totalCohesion = (totalCohesion / static_cast<float>(count + memCount)) - pos;
        if (glm::length2(totalCohesion) > 0.0f)
        {
            totalCohesion = glm::normalize(totalCohesion) * globalSpeciesParams.cohesion * 1.2f;
        }

        if (count > 0)
        {
            separation = glm::normalize(separation) * globalSpeciesParams.separation * 0.8f;
            alignment = (alignment / static_cast<float>(count) - vel) * globalSpeciesParams.alignment;
        }

        accelerations[index] += separation + alignment + totalCohesion;
    }
}

// 分割が必要か判定
bool BoidUnit::needsSplit(float splitRadius, float directionVarThresh, int maxBoids) const
{
    if ((int)positions.size() > maxBoids)
        return true;
    if (radius > splitRadius)
        return true;

    // 方向のバラつき判定
    if (positions.size() > 1)
    {
        glm::vec3 avg = glm::vec3(0.0f);
        for (const auto &vel : velocities)
            avg += glm::normalize(vel);
        avg /= static_cast<float>(velocities.size());

        float var = 0.0f;
        for (const auto &vel : velocities)
            var += glm::length(glm::normalize(vel) - avg);
        var /= static_cast<float>(velocities.size());

        if (var > directionVarThresh)
            return true;
    }
    return false;
}

// 2分割（例：最大分散軸で分割）→ 任意分割
std::vector<BoidUnit *> BoidUnit::split(int numSplits)
{
    int axis = 0;
    float maxVar = 0.0f;

    // 最大分散軸を計算
    for (int i = 0; i < 3; ++i)
    {
        float mean = 0.0f, var = 0.0f;
        for (const auto &pos : positions)
            mean += (i == 0 ? pos.x : (i == 1 ? pos.y : pos.z));
        mean /= positions.size();

        for (const auto &pos : positions)
        {
            float v = (i == 0 ? pos.x : (i == 1 ? pos.y : pos.z)) - mean;
            var += v * v;
        }

        if (var > maxVar)
        {
            maxVar = var;
            axis = i;
        }
    }

    // min, maxを求めて等間隔で分割
    float minVal = std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::lowest();
    for (const auto &pos : positions)
    {
        float v = (axis == 0 ? pos.x : (axis == 1 ? pos.y : pos.z));
        minVal = std::min(minVal, v);
        maxVal = std::max(maxVal, v);
    }

    float interval = (maxVal - minVal) / numSplits;
    std::vector<std::vector<size_t>> groups(numSplits);

    for (size_t i = 0; i < positions.size(); ++i)
    {
        float v = (axis == 0 ? positions[i].x : (axis == 1 ? positions[i].y : positions[i].z));
        int idx = std::min(numSplits - 1, int((v - minVal) / interval));
        groups[idx].push_back(i);
    }

    // 新しいBoidUnitを生成
    std::vector<BoidUnit *> result;
    for (const auto &group : groups)
    {
        if (!group.empty())
        {
            BoidUnit *unit = new BoidUnit();
            for (size_t idx : group)
            {
                unit->positions.push_back(positions[idx]);
                unit->velocities.push_back(velocities[idx]);
                unit->accelerations.push_back(accelerations[idx]);
                unit->ids.push_back(ids[idx]);
            }
            unit->computeBoundingSphere();
            result.push_back(unit);
        }
    }
    return result;
}
// k-means風クラスタリングで近いもの同士をグループ化
std::vector<BoidUnit *> BoidUnit::splitByClustering(int numClusters)
{
    if ((int)positions.size() < numClusters)
        numClusters = positions.size();
    if (numClusters < 2)
        numClusters = 2;

    // 初期中心をランダム選択
    std::vector<glm::vec3> centers;
    for (int i = 0; i < numClusters; ++i)
        centers.push_back(positions[i]); // ランダム選択を簡略化

    std::vector<int> assignments(positions.size(), 0);
    if (assignments.size() != positions.size())
    {
        std::cerr << "Error: assignments size mismatch!" << std::endl;
        assignments.resize(positions.size(), 0);
    }

    for (int iter = 0; iter < 5; ++iter) // 反復回数少なめ
    {
        // 割り当て
        for (size_t i = 0; i < positions.size(); ++i)
        {
            float minDist = std::numeric_limits<float>::max();
            int best = 0;
            for (int k = 0; k < numClusters; ++k)
            {
                float d = glm::distance(positions[i], centers[k]);
                if (d < minDist)
                {
                    minDist = d;
                    best = k;
                }
            }
            assignments[i] = best;
        }

        // 中心再計算
        std::vector<glm::vec3> newCenters(numClusters, glm::vec3(0.0f));
        std::vector<int> counts(numClusters, 0);
        for (size_t i = 0; i < positions.size(); ++i)
        {
            newCenters[assignments[i]] += positions[i];
            counts[assignments[i]]++;
        }
        for (int k = 0; k < numClusters; ++k)
        {
            if (counts[k] > 0)
                newCenters[k] = newCenters[k] / static_cast<float>(counts[k]);
            else
                newCenters[k] = centers[k];
        }
        centers = newCenters;
    }

    // グループ分け
    std::vector<std::vector<size_t>> groups(numClusters);
    for (size_t i = 0; i < positions.size(); ++i)
    {
        if (assignments[i] >= numClusters)
        {
            std::cerr << "Error: Invalid cluster assignment!" << std::endl;
            continue;
        }
        groups[assignments[i]].push_back(i);
    }

    // BoidUnit生成
    std::vector<BoidUnit *> result;
    for (const auto &group : groups)
    {
        if (!group.empty())
        {
            BoidUnit *unit = new BoidUnit();
            for (size_t idx : group)
            {
                unit->positions.push_back(positions[idx]);
                unit->velocities.push_back(velocities[idx]);
                unit->accelerations.push_back(accelerations[idx]);
                unit->ids.push_back(ids[idx]);
            }
            unit->computeBoundingSphere();
            result.push_back(unit);
        }
    }
    return result;
}

/**
 * 現在のユニットを分割し、子ノードとして配置する。
 *
 * @param maxBoids Boid の最大数
 *
 * 処理内容:
 * - 分割が必要かを判定。
 * - 必要であればクラスタリングを用いて分割し、子ノードを生成。
 * - 親ノードの情報を更新。
 *
 * 使用例:
 * - Boid の数が多すぎる場合にユニットを分割して階層構造を維持する際に使用。
 */
void BoidUnit::splitInPlace(int maxBoids)
{
    if (!needsSplit(80.0f, 0.5f, maxBoids))
        return;

    // 例: 近いもの同士で最大4グループに分割
    auto splits = splitByClustering(4);

    // 現在のユニットのデータをクリア
    positions.clear();
    velocities.clear();
    accelerations.clear();
    ids.clear();
    stresses.clear();
    speciesIds.clear();

    // 子ノードを設定
    this->children = splits;

    // 子ノードのレベルを設定
    for (auto *c : children)
    {
        c->level = this->level + 1;
    }

    // バウンディングスフィアを再計算
    computeBoundingSphere();
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
bool BoidUnit::canMergeWith(const BoidUnit &other, float mergeDist, float velThresh, float maxRadius, int maxBoids) const
{
    // 中心間の距離が閾値を超える場合は結合不可
    if (glm::distance(center, other.center) > mergeDist)
        return false;

    // 平均速度の差が閾値を超える場合は結合不可
    if (glm::length(averageVelocity - other.averageVelocity) > velThresh)
        return false;

    // Boid数の上限を超える場合は結合不可
    if ((int)positions.size() + (int)other.positions.size() > maxBoids)
        return false;

    // 結合後の中心を計算
    glm::vec3 newCenter = (center * static_cast<float>(positions.size()) +
                           other.center * static_cast<float>(other.positions.size())) /
                          static_cast<float>(positions.size() + other.positions.size());

    // 結合後の半径を計算
    float newRadius = 0.0f;
    for (const auto &pos : positions)
        newRadius = std::max(newRadius, glm::distance(newCenter, pos));
    for (const auto &pos : other.positions)
        newRadius = std::max(newRadius, glm::distance(newCenter, pos));

    // 半径が上限を超える場合は結合不可
    if (newRadius > maxRadius)
        return false;

    return true;
}
/**
 * 他のユニットを結合する。
 *
 * @param other 他の BoidUnit
 *
 * 処理内容:
 * - 他のユニットの Boid を現在のユニットに追加。
 * - バウンディングスフィアを再計算。
 *
 * 使用例:
 * - Boid の数が少ない場合にユニットを結合して階層構造を簡略化する際に使用。
 */
void BoidUnit::mergeWith(const BoidUnit &other)
{
    // 他のユニットのデータを現在のユニットに追加
    positions.insert(positions.end(), other.positions.begin(), other.positions.end());
    velocities.insert(velocities.end(), other.velocities.begin(), other.velocities.end());
    accelerations.insert(accelerations.end(), other.accelerations.begin(), other.accelerations.end());
    ids.insert(ids.end(), other.ids.begin(), other.ids.end());
    stresses.insert(stresses.end(), other.stresses.begin(), other.stresses.end());
    speciesIds.insert(speciesIds.end(), other.speciesIds.begin(), other.speciesIds.end());

    // バウンディングスフィアを再計算
    computeBoundingSphere();
}

void BoidUnit::mergeWith(BoidUnit *other, BoidUnit *parent)
{
    // 他のユニットのデータを現在のユニットに追加
    positions.insert(positions.end(), other->positions.begin(), other->positions.end());
    velocities.insert(velocities.end(), other->velocities.begin(), other->velocities.end());
    accelerations.insert(accelerations.end(), other->accelerations.begin(), other->accelerations.end());
    ids.insert(ids.end(), other->ids.begin(), other->ids.end());
    stresses.insert(stresses.end(), other->stresses.begin(), other->stresses.end());
    speciesIds.insert(speciesIds.end(), other->speciesIds.begin(), other->speciesIds.end());

    // 子ノードをクリア
    children.clear();

    // バウンディングスフィアを再計算
    computeBoundingSphere();

    // 親ノードの children から other を除去
    if (parent)
    {
        auto it = std::find(parent->children.begin(), parent->children.end(), other);
        if (it != parent->children.end())
        {
            parent->children.erase(it);
        }
    }

    // メモリ解放
    delete other;
}

// 兄弟ノード配下の全Boidに反発を加算するユーティリティ
void BoidUnit::addRepulsionToAllBoids(BoidUnit *unit, const glm::vec3 &repulsion)
{
    if (unit->isBoidUnit())
    {
        for (size_t i = 0; i < unit->positions.size(); ++i)
        {
            float d = glm::length(unit->positions[i] - unit->center);
            float w = 0.5f + 0.5f * (d / (unit->radius + 1e-5f)); // 端ほど1.0、中心0.5
            unit->accelerations[i] += repulsion * w;
        }
    }
    else
    {
        for (auto *c : unit->children)
        {
            addRepulsionToAllBoids(c, repulsion);
        }
    }
}