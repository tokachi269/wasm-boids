#include "boid_unit.h"
#include "boids_tree.h"
#include <algorithm>
#include <queue>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

bool BoidUnit::isBoidUnit() const { return children.empty(); }

void BoidUnit::computeBoundingSphere()
{
    if (isBoidUnit())
    {
        if (boids.empty())
            return;
        center = Vec3();
        for (const auto &b : boids)
            center = center + b.position;
        center = center / boids.size();
        radius = 0.0f;
        for (const auto &b : boids)
        {
            float d = center.distance(b.position);
            if (d > radius)
                radius = d;
        }
    }
    else
    {
        center = Vec3();
        for (const auto &c : children)
            center = center + c->center;
        center = center / children.size();
        radius = 0.0f;
        for (const auto &c : children)
        {
            float d = center.distance(c->center) + c->radius;
            if (d > radius)
                radius = d;
        }
    }
}

BoidStats BoidUnit::computeBoidStats(Boid &self, const std::vector<Boid> &others) const
{
    BoidStats s;
    // 最大近傍数と吸引強度をパラメータから取得
    int Nu = globalSpeciesParams.maxNeighbors;
    float lambda = globalSpeciesParams.lambda;

    // 距離とBoid参照のペアを作成
    std::vector<std::pair<float, const Boid *>> neighbors;
    for (const auto &n : others)
    {
        if (n.id == self.id)
            continue;
        float dist = self.position.distance(n.position);
        neighbors.emplace_back(dist, &n);
    }
    // 距離でソートし、近い順にNu個だけ使う
    std::sort(neighbors.begin(), neighbors.end(),
              [](const auto &a, const auto &b)
              { return a.first < b.first; });

    int used = 0;
    for (const auto &[dist, nPtr] : neighbors)
    {
        if (used >= Nu)
            break;
        const Boid &n = *nPtr;
        // 距離減衰重み
        float weight = std::exp(-lambda * dist);

        if (n.speciesId == self.speciesId)
        {
            s.sumVelocity = s.sumVelocity + n.velocity * weight;
            s.sumPosition = s.sumPosition + n.position * weight;
            Vec3 diff = self.position - n.position;
            if (dist > 0.01f)
            {
                s.separation = s.separation + (diff / (dist * dist)) * weight;
                self.stress += (n.stress - self.stress) * 0.1f * weight;
            }
            s.count++;
        }
        else
        {
            Vec3 diff = self.position - n.position;
            if (dist > 0.01f)
            {
                s.separation = s.separation + (diff / (dist * dist)) * 2.0f * weight;
            }
        }
        used++;
    }
    return s;
}

void BoidUnit::applyInterUnitInfluence(BoidUnit *other)
{
    if (!boids.empty() && !other->boids.empty())
    {
        // 葉ノード同士はboidごとに詳細計算
        for (Boid &a : boids)
        {
            Vec3 sumVelocity, sumPosition, separation;
            int count = 0;
            for (const Boid &b : other->boids)
            {
                float dist = a.position.distance(b.position);
                if (dist < 40.0f && dist > 0.01f)
                {
                    float influence = std::max(0.0f, 1.0f - (dist / 40.0f));
                    sumVelocity += b.velocity * influence;
                    sumPosition += b.position * influence;
                    Vec3 diff = a.position - b.position;
                    separation += (diff / (dist * dist)) * influence;
                    count += influence;
                }
            }
            if (count > 0)
            {
                a.acceleration += ((sumVelocity / count - a.velocity) * globalSpeciesParams.alignment);
                a.acceleration += ((sumPosition / count - a.position) * globalSpeciesParams.cohesion);
                a.acceleration += (separation * globalSpeciesParams.separation);
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
        for (auto *c : children)
        {
            c->applyInterUnitInfluence(other);
        }
    }
    else
    {
        // 両方とも中間ノード → 代表値で近似計算
        float dist = center.distance(other->center);

        // 代表値近似の影響をunitサイズで減衰
        float rangeA = globalSpeciesParams.cohesionRange;
        float rangeB = globalSpeciesParams.cohesionRange;
        if (radius > rangeA || other->radius > rangeB)
            return; // 大きすぎるunitは代表値近似を適用しない

        if (dist < 400.0f && dist > 0.01f)
        {
            float influence = 1.0f / (dist * dist + 1.0f);
            // unitが大きいほど影響を急激に減衰
            float scaleA = std::clamp(1.0f - (radius / rangeA) * (radius / rangeA), 0.0f, 1.0f);
            float scaleB = std::clamp(1.0f - (other->radius / rangeB) * (other->radius / rangeB), 0.0f, 1.0f);
            float scale = influence * scaleA * scaleB * 0.3f;

            Vec3 align = (other->averageVelocity - averageVelocity) * globalSpeciesParams.alignment;
            Vec3 cohes = (other->center - center) * globalSpeciesParams.cohesion;
            Vec3 separation = (other->center - center) * globalSpeciesParams.separation;
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
                b->acceleration += align * scale;
                b->acceleration += cohes * scale;
                b->acceleration += separation * scale;
            }
            for (Boid *b : otherBoids)
            {
                b->acceleration -= align * scale;
                b->acceleration -= cohes * scale;
                b->acceleration -= separation * scale;
            }
        }
    }
}

void BoidUnit::updateRecursive(float dt)
{
    if (isBoidUnit())
    {
        // 1. まず全Boidの合計値を計算
        Vec3 totalVelocity, totalPosition;
        for (const auto &b : boids)
        {
            totalVelocity += b.velocity;
            totalPosition += b.position;
        }

        // 2. 各Boidごとに「自分以外の合計」を使って計算
        for (auto &b : boids)
        {
            int n = (int)boids.size() - 1;
            if (n <= 0)
                continue;

            Vec3 sumVelocity, sumPosition, separation;
            int alignCount = 0, cohesCount = 0, separCount = 0;
            float stress = b.stress;

            // 距離閾値
            float separRange = globalSpeciesParams.separationRange;
            float alignRange = globalSpeciesParams.alignmentRange;
            float cohesRange = globalSpeciesParams.cohesionRange;

            // --- ここから最適化 ---
            // 近傍候補を距離付きで収集
            std::vector<std::pair<float, const Boid*>> neighbors;
            for (const auto &nboid : boids)
            {
                if (nboid.id == b.id) continue;
                float dist = b.position.distance(nboid.position);
                neighbors.emplace_back(dist, &nboid);
            }

            // 最大近傍数と減衰パラメータ
            int Nu = globalSpeciesParams.maxNeighbors;
            float lambda = globalSpeciesParams.lambda;

            // 近い順にNu個だけ使う
            if ((int)neighbors.size() > Nu) {
                std::partial_sort(neighbors.begin(), neighbors.begin() + Nu, neighbors.end(),
                    [](const auto &a, const auto &b) { return a.first < b.first; });
                neighbors.resize(Nu);
            }

            for (const auto &[dist, nPtr] : neighbors)
            {
                const Boid &nboid = *nPtr;
                float weight = std::exp(-lambda * dist);

                // 分離
                if (dist < separRange && dist > 0.01f)
                {
                    Vec3 diff = b.position - nboid.position;
                    separation += (diff / (dist * dist)) * weight;
                    separCount++;
                    stress += (nboid.stress - b.stress) * 0.1f * weight;
                }
                // 整列
                if (dist < alignRange)
                {
                    sumVelocity += nboid.velocity * weight;
                    alignCount++;
                }
                // 凝集
                if (dist < cohesRange)
                {
                    sumPosition += nboid.position * weight;
                    cohesCount++;
                }
            }
            // --- ここまで最適化 ---

            // ルール適用（既存のまま）
            Vec3 align = alignCount > 0 ? ((sumVelocity / alignCount - b.velocity) * globalSpeciesParams.alignment) : Vec3();
            Vec3 cohesTarget = cohesCount > 0 ? ((sumPosition / cohesCount - b.position).normalized()) : Vec3();
            Vec3 forward = b.velocity.normalized();

            float boost = 1.0f + b.stress * 0.8f;
            Vec3 separ = separation * (globalSpeciesParams.separation + b.stress * 0.4f);

            // --- 既存コードの一部を最適化 ---

            // 例：進行方向ベクトルの正規化と長さ判定
            float forwardLenSq = forward.lengthSq();
            Vec3 forwardNorm;
            if (forwardLenSq > 0.000001f) {
                forwardNorm = forward / std::sqrt(forwardLenSq);
            } else {
                forwardNorm = Vec3();
            }

            // 以降は forwardNorm を使い回す
            if (cohesCount > 0 && forwardLenSq > 0.000001f)
            {
                float maxCohesionAngle = 0.1f;
                float dot = forwardNorm.dot(cohesTarget);
                float angle = acos(std::clamp(dot, -1.0f, 1.0f));
                if (angle > maxCohesionAngle)
                {
                    Vec3 axis = forwardNorm.cross(cohesTarget).normalized();
                    cohesTarget = Vec3::rotateVector(forwardNorm, axis, maxCohesionAngle);
                }
                Vec3 cohes = (cohesTarget - forwardNorm) * globalSpeciesParams.cohesion * (1.0f - b.stress);
                b.acceleration += cohes * boost;
            }
            else
            {
                Vec3 cohes = cohesCount > 0 ? ((sumPosition / cohesCount - b.position) * globalSpeciesParams.cohesion * (1.0f - b.stress)) : Vec3();
                b.acceleration += cohes * boost;
            }

            // ...他の箇所も .normalized() や .length() を複数回呼ばず、変数にキャッシュして使い回す...

            // 進行方向ベクトル
            Vec3 forwardVec = b.velocity;
            float forwardVecLenSq = forwardVec.lengthSq();
            Vec3 forwardVecNorm;
            if (forwardVecLenSq > 0.000001f) {
                forwardVecNorm = forwardVec / std::sqrt(forwardVecLenSq);
            } else {
                forwardVecNorm = Vec3();
            }
            // XZ平面上の進行方向
            Vec3 flatForward = Vec3(forwardVecNorm.x, 0.0f, forwardVecNorm.z);
            float flatForwardLenSq = flatForward.lengthSq();
            if (flatForwardLenSq > 0.000001f)
            {
                flatForward = flatForward / std::sqrt(flatForwardLenSq);
                // 上下方向（y成分）を減らす補正ベクトル
                Vec3 flatten = (flatForward - forwardVecNorm) * 0.1f; // 係数は調整
                b.acceleration += flatten;
            }

            float boundary = 200.0f;
            if (b.position.length() > boundary)
            {
                Vec3 toOrigin = (Vec3(0, 0, 0) - b.position) * 0.002f;
                b.acceleration += toOrigin;
            }
            // jitter（微小なランダムノイズ）を加える
            float jitterStrength = 0.1f;
            b.acceleration.x += ((float)rand() / float(RAND_MAX) - 0.5f) * jitterStrength;
            b.acceleration.y += ((float)rand() / 2147483647.0f - 0.5f) * jitterStrength;
            b.acceleration.z += ((float)rand() / 2147483647.0f - 0.5f) * jitterStrength;

            /** 速度の更新 */
            Vec3 desiredVelocity = b.velocity + b.acceleration * dt;
            float maxTurnAngle = globalSpeciesParams.maxTurnAngle;
            float speed = desiredVelocity.length();
            if (speed > 0.001f)
            {
                Vec3 oldDir = b.velocity.normalized();
                Vec3 newDir = desiredVelocity.normalized();
                float angle = acos(std::clamp(oldDir.dot(newDir), -1.0f, 1.0f));
                if (angle > maxTurnAngle)
                {
                    // oldDirからnewDirへmaxTurnAngleだけ回転
                    Vec3 axis = oldDir.cross(newDir).normalized();
                    // 回転行列またはSLERPで補間（Vec3型なら自作関数が必要）
                    newDir = Vec3::rotateVector(oldDir, axis, maxTurnAngle);
                    b.velocity = newDir * speed;
                }
                else
                {
                    b.velocity = desiredVelocity;
                }
            }

            /** 最大速度の制限 */
            float maxSpeed = globalSpeciesParams.maxSpeed;
            if (speed > maxSpeed)
            {
                b.velocity = b.velocity * (maxSpeed / speed);
            }

            // ...既存の最大速度制限の直後に追加...
            float minSpeed = globalSpeciesParams.minSpeed;
            if (speed < minSpeed && speed > 0.0001f)
            {
                b.velocity = b.velocity * (minSpeed / speed);
            }

            // 速度方向に微小な推進力
            if (speed > 0.001f)
            {
                b.acceleration += (b.velocity / speed) * 0.01f;
            }

            /** 位置の更新 */
            b.position += b.velocity * dt;

            /** ストレスの減少 */
            b.stress = std::max(0.0f, b.stress - 0.005f);
        }

        // 中心・平均速度の再計算はここで一度だけ
        center = averageVelocity = Vec3();
        for (const auto &b : boids)
        {
            center = center + b.position;
            averageVelocity = averageVelocity + b.velocity;
        }
        if (!boids.empty())
        {
            center = center / boids.size();
            averageVelocity = averageVelocity / boids.size();
        }
    }
    else
    {
        for (auto &c : children)
            c->updateRecursive(dt);
        for (size_t i = 0; i < children.size(); ++i)
        {
            for (size_t j = i + 1; j < children.size(); ++j)
            {
                children[i]->applyInterUnitInfluence(children[j]);
                children[j]->applyInterUnitInfluence(children[i]);
            }
        }
        center = averageVelocity = Vec3();
        for (const auto &c : children)
        {
            center = center + c->center;
            averageVelocity = averageVelocity + c->averageVelocity;
        }
        if (!children.empty())
        {
            center = center / children.size();
            averageVelocity = averageVelocity / children.size();
        }
    }
    computeBoundingSphere();
}

// 分割が必要か判定
bool BoidUnit::needsSplit(float splitRadius, float directionVarThresh, int maxBoids) const
{
    if ((int)boids.size() > maxBoids)
        return true;
    if (radius > splitRadius)
        return true;
    // 密度基準を追加
    float density = boids.size() / (4.0f / 3.0f * M_PI * std::max(radius, 1e-3f) * std::max(radius, 1e-3f) * std::max(radius, 1e-3f));
    if (density < 0.01f) // 密度が低すぎる場合も分割
        return true;
    // 方向のバラつき判定
    if (boids.size() > 1)
    {
        Vec3 avg = Vec3();
        for (const auto &b : boids)
            avg += b.velocity.normalized();
        avg = avg / boids.size();
        float var = 0.0f;
        for (const auto &b : boids)
            var += (b.velocity.normalized() - avg).length();
        var /= boids.size();
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
    std::vector<Vec3> centers;
    std::vector<Boid> seeds = boids;
    for (int i = 0; i < numClusters; ++i)
        centers.push_back(seeds[i].position);

    std::vector<int> assignments(boids.size(), 0);
    for (int iter = 0; iter < 5; ++iter)
    { // 反復回数少なめ
        // 割り当て
        for (size_t i = 0; i < boids.size(); ++i)
        {
            float minDist = 1e9;
            int best = 0;
            for (int k = 0; k < numClusters; ++k)
            {
                float d = boids[i].position.distance(centers[k]);
                if (d < minDist)
                {
                    minDist = d;
                    best = k;
                }
            }
            assignments[i] = best;
        }
        // 中心再計算
        std::vector<Vec3> newCenters(numClusters, Vec3());
        std::vector<int> counts(numClusters, 0);
        for (size_t i = 0; i < boids.size(); ++i)
        {
            newCenters[assignments[i]] += boids[i].position;
            counts[assignments[i]]++;
        }
        for (int k = 0; k < numClusters; ++k)
        {
            if (counts[k] > 0)
                newCenters[k] = newCenters[k] / counts[k];
            else
                newCenters[k] = centers[k];
        }
        centers = newCenters;
    }
    // グループ分け
    std::vector<std::vector<Boid>> groups(numClusters);
    for (size_t i = 0; i < boids.size(); ++i)
    {
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

// 親ノードから自身を分割する
void BoidUnit::splitInPlace(int maxBoids)
{
    if (!needsSplit(40.0f, 0.5f, maxBoids))
        return;
    // 例: 近いもの同士で最大4グループに分割
    auto splits = splitByClustering(4);
    this->boids.clear();
    this->children = splits;
    for (auto *c : children)
        c->level = this->level + 1;
    computeBoundingSphere();
}

// 結合可能か判定
bool BoidUnit::canMergeWith(const BoidUnit &other, float mergeDist, float velThresh, float maxRadius, int maxBoids) const
{
    if (center.distance(other.center) > mergeDist)
        return false;
    if ((averageVelocity - other.averageVelocity).length() > velThresh)
        return false;
    // Boid数上限チェック
    if ((int)boids.size() + (int)other.boids.size() > maxBoids)
        return false;
    // 結合後の半径
    Vec3 newCenter = (center * boids.size() + other.center * other.boids.size()) / (boids.size() + other.boids.size());
    float newRadius = 0.0f;
    for (const auto &b : boids)
        newRadius = std::max(newRadius, newCenter.distance(b.position));
    for (const auto &b : other.boids)
        newRadius = std::max(newRadius, newCenter.distance(b.position));
    if (newRadius > maxRadius)
        return false;
    return true;
}

// 結合
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