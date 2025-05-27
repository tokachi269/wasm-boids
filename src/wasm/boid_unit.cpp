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
        if (boids.empty())
            return;
        center = glm::vec3();
        for (const auto &b : boids)
            center = center + b.position;
        center = center / static_cast<float>(boids.size());

        // 平均距離と分散を計算
        float sum = 0.0f, sum2 = 0.0f;
        for (const auto &b : boids)
        {
            float d = glm::distance(center, b.position);
            sum += d;
            sum2 += d * d;
        }
        float mean = sum / static_cast<float>(boids.size());
        float var = sum2 / boids.size() - mean * mean;
        float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

        // 平均+α×標準偏差（例: α=1.5）でradiusを決定
        radius = mean + 1.0f * stddev;
    }
    else
    {
        center = glm::vec3();
        for (const auto &c : children)
            center = center + c->center;
        center = center / static_cast<float>(children.size());

        // 子ノード中心までの平均距離＋子ノードの半径の平均＋α×標準偏差
        float sum = 0.0f, sum2 = 0.0f;
        for (const auto &c : children)
        {
            float d = glm::distance(center, c->center) + c->radius;
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
    if (!boids.empty() && !other->boids.empty())
    {
        // 葉ノード同士はboidごとに詳細計算
        for (Boid &a : boids)
        {
            glm::vec3 sumVelocity = glm::vec3(0.0f);
            glm::vec3 sumPosition = glm::vec3(0.0f);
            glm::vec3 separation = glm::vec3(0.0f);
            int count = 0;
            for (const Boid &b : other->boids)
            {
                float distSq = glm::length2(a.position - b.position); // 距離の二乗を計算
                if (distSq < 2500.0f && distSq > 0.0001f)             // 50.0f^2 = 2500.0f
                {
                    float dist = std::sqrt(distSq); // 必要な場合のみ平方根を計算
                    // 例: 葉ノード同士の影響計算
                    float influence = std::max(0.0f, 1.0f - (dist / 40.0f));
                    sumVelocity += b.velocity * influence;
                    sumPosition += b.position * influence;
                    glm::vec3 diff = a.position - b.position;
                    separation += (diff / (dist * dist + 1.0f)) * influence;
                    count += influence;
                }
            }
            if (count > 0)
            {
                a.acceleration += ((sumVelocity / static_cast<float>(count) - a.velocity) * globalSpeciesParams.alignment);
                a.acceleration += ((sumPosition / static_cast<float>(count) - a.position) * (globalSpeciesParams.cohesion * 0.5f)); // 凝集の影響を50%に
                a.acceleration += (separation * (globalSpeciesParams.separation * 0.5f));                                           // 分離の影響を50%に
            }
        }
    }
    else if (!boids.empty() && other->boids.empty())
    {
        // 片方が葉ノード、片方が中間ノード
        for (auto *c : other->children)
        {
            applyInterUnitInfluence(c);
        }
    }
    else if (boids.empty() && !other->boids.empty())
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

            // このノード配下の全Boid
            std::vector<Boid *> allBoids;
            std::queue<BoidUnit *> q;
            q.push(this);
            while (!q.empty())
            {
                BoidUnit *u = q.front();
                q.pop();
                if (!u->boids.empty())
                {
                    for (Boid &b : u->boids)
                        allBoids.push_back(&b);
                }
                else
                {
                    for (BoidUnit *c : u->children)
                        q.push(c);
                }
            }

            // 相手ノード配下の全Boid
            std::vector<Boid *> otherBoids;
            q.push(other);
            while (!q.empty())
            {
                BoidUnit *u = q.front();
                q.pop();
                if (!u->boids.empty())
                {
                    for (Boid &b : u->boids)
                        otherBoids.push_back(&b);
                }
                else
                {
                    for (BoidUnit *c : u->children)
                        q.push(c);
                }
            }

            // 影響を加算
            for (Boid *b : allBoids)
            {
                // 方向ベクトルを直接回転させる（トルク的な影響を加速度加算ではなく回転で表現）
                glm::vec3 fwd = glm::normalize(b->velocity);
                glm::vec3 target = glm::normalize(other->center - center);
                float dot = std::clamp(glm::dot(fwd, target), -1.0f, 1.0f);
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
                        b->velocity = newDir * static_cast<float>(b->velocity.length());
                    }
                }

                // 回転トルクに基づく加速度の調整
                glm::vec3 torque = axis * angle * torqueStrength;
                b->acceleration += torque;
            }
            for (Boid *b : otherBoids)
            {
                glm::vec3 fwd = glm::normalize(b->velocity);
                glm::vec3 target = glm::normalize(center - other->center);
                float dot = std::clamp(glm::dot(fwd, target), -1.0f, 1.0f);
                float angle = acos(dot);
                if (angle > 1e-4f)
                {
                    glm::vec3 axis = glm::cross(fwd, target);
                    float axisLen = axis.length();
                    if (axisLen > 1e-4f)
                    {
                        axis = axis / axisLen;
                        float torqueStrength = 0.02f;
                        float rotateAngle = std::min(angle, torqueStrength * scale);
                        rotateAngle = std::min(rotateAngle, globalSpeciesParams.maxTurnAngle);
                        glm::vec3 newDir = glm::rotate(fwd, rotateAngle, axis);
                        b->velocity = newDir * static_cast<float>(b->velocity.length());
                    }
                }

                // b->acceleration -= align * scale;
                b->acceleration -= cohes * scale;
                b->acceleration -= separation * scale;
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
            for (size_t i = 0; i < current->boids.size(); ++i)
            {
                computeBoidInteraction(current->boids[i], current->boids, dt);
            }
        }
        else
        {
            for (auto &child : current->children)
                stack.push(child);

            for (size_t i = 0; i < current->children.size(); ++i)
            {
                for (size_t j = i + 1; j < current->children.size(); ++j)
                {
                    current->children[i]->applyInterUnitInfluence(current->children[j]);
                }
            }
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
            for (Boid &b : current->boids)
            {
                glm::vec3 desiredVelocity = b.velocity + b.acceleration * dt;
                glm::vec3 oldDir = glm::normalize(b.velocity);
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
                b.velocity = newDir * finalSpeed;
                b.position += b.velocity * dt;
                b.stress = glm::max(0.0f, b.stress - 0.005f);
            }
        }
        else
        {
            for (auto &child : current->children)
                stack.push(child);
        }
    }
}

void BoidUnit::computeBoidInteraction(Boid &b, const std::vector<Boid> &boids, float dt)
{
    glm::vec3 separation = glm::vec3(0.0f);
    glm::vec3 alignment = glm::vec3(0.0f);
    glm::vec3 cohesion = glm::vec3(0.0f);
    glm::vec3 memCohesion = glm::vec3(0.0f);

    int count = 0;
    int memCount = 0;

    std::unordered_set<int> visibleIds;
    visibleIds.reserve(boids.size());

    // id → Boidポインタのマップを構築
    std::unordered_map<int, const Boid*> idToBoidMap;
    idToBoidMap.reserve(boids.size());
    for (const auto& other : boids)
    {
        idToBoidMap[other.id] = &other;
    }

    for (const auto &other : boids)
    {
        if (b.id == other.id)
            continue;

        glm::vec3 diff = b.position - other.position;
        float distSq = glm::dot(diff, diff);

        if (distSq < 2500.0f && distSq > 0.0001f)
        {
            float dist = sqrtf(distSq);

            float separationWeight = glm::clamp(1.0f - (dist / 50.0f), 0.0f, 1.0f);
            separation += diff * (separationWeight * globalSpeciesParams.separation);

            float cohesionWeight = glm::clamp(dist / 50.0f, 0.0f, 1.0f);
            cohesion += other.position * cohesionWeight;

            alignment += other.velocity;
            count++;

            visibleIds.insert(other.id);
            b.cohesionMemory[other.id] = 0.0f;
        }
    }

    if (frameCount % 5 == 0)
    {
        for (auto it = b.cohesionMemory.begin(); it != b.cohesionMemory.end();)
        {
            if (visibleIds.find(it->first) == visibleIds.end())
            {
                it->second += dt;
                if (it->second > globalSpeciesParams.tau)
                {
                    it = b.cohesionMemory.erase(it);
                    continue;
                }
            }
            ++it;
        }
    }

    const size_t maxCohesionMemorySize = 100;
    if (b.cohesionMemory.size() > maxCohesionMemorySize)
    {
        auto oldest = std::min_element(
            b.cohesionMemory.begin(), b.cohesionMemory.end(),
            [](const auto &a, const auto &b)
            { return a.second > b.second; });
        if (oldest != b.cohesionMemory.end())
        {
            b.cohesionMemory.erase(oldest);
        }
    }

    for (const auto &mem : b.cohesionMemory)
    {
        auto it = idToBoidMap.find(mem.first);
        if (it != idToBoidMap.end())
        {
            memCohesion += it->second->position;
            memCount++;
        }
    }

    if (count + memCount > 0)
    {
        glm::vec3 totalCohesion = cohesion + memCohesion;
        totalCohesion = (totalCohesion / static_cast<float>(count + memCount)) - b.position;
        if (glm::length2(totalCohesion) > 0.0f)
        {
            totalCohesion = glm::normalize(totalCohesion) * globalSpeciesParams.cohesion * 1.2f;
        }

        if (count > 0)
        {
            separation = glm::normalize(separation) * globalSpeciesParams.separation * 0.8f;
            alignment = (alignment / static_cast<float>(count) - b.velocity) * globalSpeciesParams.alignment;
        }

        b.acceleration += separation + alignment + totalCohesion;
    }
}


// 分割が必要か判定
bool BoidUnit::needsSplit(float splitRadius, float directionVarThresh, int maxBoids) const
{
    if ((int)boids.size() > maxBoids)
        return true;
    if (radius > splitRadius)
        return true;
    // 方向のバラつき判定
    if (boids.size() > 1)
    {
        glm::vec3 avg = glm::vec3();
        for (const auto &b : boids)
            avg += glm::normalize(b.velocity);
        avg = avg / static_cast<float>(boids.size());
        float var = 0.0f;
        for (const auto &b : boids)
            var += glm::length(glm::normalize(b.velocity) - avg);
        var /= static_cast<float>(boids.size());
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
    for (int i = 0; i < 3; ++i)
    {
        float mean = 0, var = 0;
        for (const auto &b : boids)
            mean += (i == 0 ? b.position.x : (i == 1 ? b.position.y : b.position.z));
        mean /= boids.size();
        for (const auto &b : boids)
        {
            float v = (i == 0 ? b.position.x : (i == 1 ? b.position.y : b.position.z)) - mean;
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
    for (const auto &b : boids)
    {
        float v = (axis == 0 ? b.position.x : (axis == 1 ? b.position.y : b.position.z));
        minVal = std::min(minVal, v);
        maxVal = std::max(maxVal, v);
    }
    float interval = (maxVal - minVal) / numSplits;
    std::vector<std::vector<Boid>> groups(numSplits);
    for (const auto &b : boids)
    {
        float v = (axis == 0 ? b.position.x : (axis == 1 ? b.position.y : b.position.z));
        int idx = std::min(numSplits - 1, int((v - minVal) / interval));
        groups[idx].push_back(b);
    }
    std::vector<BoidUnit *> result;
    for (auto &group : groups)
    {
        if (!group.empty())
        {
            BoidUnit *unit = new BoidUnit();
            unit->boids = group;
            unit->computeBoundingSphere();
            result.push_back(unit);
        }
    }
    return result;
}

// k-means風クラスタリングで近いもの同士をグループ化
std::vector<BoidUnit *> BoidUnit::splitByClustering(int numClusters)
{
    if ((int)boids.size() < numClusters)
        numClusters = boids.size();
    if (numClusters < 2)
        numClusters = 2;

    // 初期中心をランダム選択
    std::vector<glm::vec3> centers;
    std::vector<Boid> seeds = boids;
    for (int i = 0; i < numClusters; ++i)
        centers.push_back(seeds[i].position);

    std::vector<int> assignments(boids.size(), 0);
    if (assignments.size() != boids.size())
    {
        std::cerr << "Error: assignments size mismatch!" << std::endl;
        assignments.resize(boids.size(), 0);
    }

    for (int iter = 0; iter < 5; ++iter)
    { // 反復回数少なめ
        // 割り当て

        for (size_t i = 0; i < boids.size(); ++i)
        {
            float minDist = 1e9;
            int best = 0;
            for (int k = 0; k < numClusters; ++k)
            {
                float d = glm::distance(boids[i].position, centers[k]);
                if (d < minDist)
                {
                    minDist = d;
                    best = k;
                }
            }
            assignments[i] = best;
        }
        // 中心再計算
        std::vector<glm::vec3> newCenters(numClusters, glm::vec3());
        std::vector<int> counts(numClusters, 0);
        for (size_t i = 0; i < boids.size(); ++i)
        {
            newCenters[assignments[i]] += boids[i].position;
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
    std::vector<std::vector<Boid>> groups(numClusters);
    if (groups.size() != static_cast<size_t>(numClusters))
    {
        std::cerr << "Error: groups size mismatch!" << std::endl;
        groups.resize(numClusters);
    }
    for (size_t i = 0; i < boids.size(); ++i)
    {
        if (assignments[i] >= numClusters)
        {
            std::cerr << "Error: Invalid cluster assignment!" << std::endl;
            continue;
        }
        groups[assignments[i]].push_back(boids[i]);
    }
    // BoidUnit生成
    std::vector<BoidUnit *> result;
    for (auto &group : groups)
    {
        if (!group.empty())
        {
            BoidUnit *unit = new BoidUnit();
            unit->boids = group;
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
    this->boids.clear();
    this->children = splits;
    for (auto *c : children)
        c->level = this->level + 1;
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
    if (glm::distance(center, other.center) > mergeDist)
        return false;
    if (glm::length(averageVelocity - other.averageVelocity) > velThresh)
        return false;
    // Boid数上限チェック
    if ((int)boids.size() + (int)other.boids.size() > maxBoids)
        return false;
    // 結合後の半径
    glm::vec3 newCenter = (center * static_cast<float>(boids.size()) + other.center * static_cast<float>(other.boids.size())) / static_cast<float>(boids.size() + other.boids.size());
    float newRadius = 0.0f;
    for (const auto &b : boids)
        newRadius = std::max(newRadius, glm::distance(newCenter, b.position));
    for (const auto &b : other.boids)
        newRadius = std::max(newRadius, glm::distance(newCenter, b.position));
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
    boids.insert(boids.end(), other.boids.begin(), other.boids.end());
    computeBoundingSphere();
}

// 結合
void BoidUnit::mergeWith(BoidUnit *other, BoidUnit *parent)
{
    boids.insert(boids.end(), other->boids.begin(), other->boids.end());
    children.clear();
    computeBoundingSphere();
    // 親ノードのchildrenからotherを除去
    if (parent)
    {
        auto it = std::find(parent->children.begin(), parent->children.end(), other);
        if (it != parent->children.end())
            parent->children.erase(it);
    }
    // メモリ解放
    delete other;
}

// 兄弟ノード配下の全Boidに反発を加算するユーティリティ
void BoidUnit::addRepulsionToAllBoids(BoidUnit *unit, const glm::vec3 &repulsion)
{
    if (unit->isBoidUnit())
    {
        for (auto &b : unit->boids)
        {
            float d = (b.position - unit->center).length();
            float w = 0.5f + 0.5f * (d / (unit->radius + 1e-5f)); // 端ほど1.0、中心0.5
            b.acceleration += repulsion * w;
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