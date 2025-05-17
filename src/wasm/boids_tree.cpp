#include "vec3.h"
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <random>
#include <emscripten/bind.h>
#include <iostream>

using namespace emscripten;

struct SpeciesParams
{
    float cohesion = 0.01f;
    float separation = 0.1f;
    float alignment = 0.05f;
    float maxSpeed = 1.0f;
    float minSpeed = 0.1f;
    float maxTurnAngle = 0.01f;
    float separationRange = 10.0f;
    float alignmentRange = 30.0f;
    float cohesionRange = 50.0f;
};

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

struct Boid
{
    Vec3 position, velocity, acceleration;
    int id = 0;
    float stress = 0.0f;
    int speciesId = 0;
};

struct BoidStats
{
    Vec3 sumVelocity, sumPosition, separation;
    int count = 0;
};

class BoidUnit
{
public:
    std::vector<Boid> boids;
    std::vector<BoidUnit *> children;
    Vec3 center, averageVelocity;
    float radius = 0.0f;
    int level = 0;

    bool isBoidUnit() const { return children.empty(); }

    void computeBoundingSphere()
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

    BoidStats computeBoidStats(Boid &self, const std::vector<Boid> &others) const
    {
        BoidStats s;
        for (const auto &n : others)
        {
            if (n.id == self.id)
                continue;
            if (n.speciesId == self.speciesId)
            {
                s.sumVelocity = s.sumVelocity + n.velocity;
                s.sumPosition = s.sumPosition + n.position;
                Vec3 diff = self.position - n.position;
                float dist = self.position.distance(n.position);
                if (dist > 0.01f)
                {
                    s.separation = s.separation + (diff / (dist * dist));
                    self.stress += (n.stress - self.stress) * 0.1f;
                }
                s.count++;
            }
            else
            {
                Vec3 diff = self.position - n.position;
                float dist = self.position.distance(n.position);
                if (dist > 0.01f)
                {
                    s.separation = s.separation + (diff / (dist * dist)) * 2.0f;
                }
            }
        }
        return s;
    }

    void applyInterUnitInfluence(BoidUnit *other)
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
                        // 例: 葉ノード同士の影響計算
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
            if (dist < 400.0f && dist > 0.01f)
            {
                float influence = 1.0f / (dist * dist + 1.0f); // 距離減衰
                float scale = influence;

                Vec3 diff = center - other->center;
                Vec3 separation = diff / (dist * dist) * globalSpeciesParams.separation;
                Vec3 align = (other->averageVelocity - averageVelocity) * globalSpeciesParams.alignment;
                Vec3 cohes = (other->center - center) * globalSpeciesParams.cohesion;

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

    void updateRecursive(float dt = 1.0f)
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

                // 距離閾値（例: separation=10, alignment=30, cohesion=50）
                float separRange = globalSpeciesParams.separationRange;
                float alignRange = globalSpeciesParams.alignmentRange;
                float cohesRange = globalSpeciesParams.cohesionRange;

                for (const auto &nboid : boids)
                {
                    if (nboid.id == b.id)
                        continue;
                    float dist = b.position.distance(nboid.position);

                    // 分離
                    if (dist < separRange && dist > 0.01f)
                    {
                        Vec3 diff = b.position - nboid.position;
                        separation += (diff / (dist * dist));
                        separCount++;
                        stress += (nboid.stress - b.stress) * 0.1f;
                    }
                    // 整列
                    if (dist < alignRange)
                    {
                        sumVelocity += nboid.velocity;
                        alignCount++;
                    }
                    // 凝集
                    if (dist < cohesRange)
                    {
                        sumPosition += nboid.position;
                        cohesCount++;
                    }
                }

                // ルール適用
                Vec3 align = alignCount > 0 ? ((sumVelocity / alignCount - b.velocity) * globalSpeciesParams.alignment) : Vec3();
                Vec3 cohesTarget = cohesCount > 0 ? ((sumPosition / cohesCount - b.position).normalized()) : Vec3();
                Vec3 forward = b.velocity.normalized();

                float boost = 1.0f + b.stress * 0.8f;
                Vec3 separ = separation * (globalSpeciesParams.separation + b.stress * 0.4f);

                if (cohesCount > 0 && forward.length() > 0.001f)
                {
                    float maxCohesionAngle = 0.01f; // 最大で何ラジアン曲げるか（例: 0.2 ≒ 11度）
                    float angle = acos(std::clamp(forward.dot(cohesTarget), -1.0f, 1.0f));
                    if (angle > maxCohesionAngle)
                    {
                        Vec3 axis = forward.cross(cohesTarget).normalized();
                        cohesTarget = Vec3::rotateVector(forward, axis, maxCohesionAngle);
                    }
                    // 進行方向に一定角度だけ中心方向に寄せたベクトルをcohesionとして加える
                    Vec3 cohes = (cohesTarget - forward) * globalSpeciesParams.cohesion * (1.0f - b.stress);
                    b.acceleration += cohes * boost;
                }
                else
                {
                    // 通常のcohesion
                    Vec3 cohes = cohesCount > 0 ? ((sumPosition / cohesCount - b.position) * globalSpeciesParams.cohesion * (1.0f - b.stress)) : Vec3();
                    b.acceleration += cohes * boost;
                }
                b.acceleration += align * boost;
                b.acceleration += separ * boost;

                // 進行方向ベクトル
                Vec3 forwardVec = b.velocity.normalized();
                // XZ平面上の進行方向
                Vec3 flatForward = Vec3(forwardVec.x, 0.0f, forwardVec.z);
                if (flatForward.length() > 0.001f)
                {
                    flatForward = flatForward.normalized();
                    // 上下方向（y成分）を減らす補正ベクトル
                    Vec3 flatten = (flatForward - forwardVec) * 0.1f; // 係数は調整
                    b.acceleration += flatten;
                }

                // 画面中心に戻る力
                Vec3 toOrigin = (Vec3(0, 0, 0) - b.position) * 0.01f;
                b.acceleration += toOrigin;

                // jitter（微小なランダムノイズ）を加える
                float jitterStrength = 0.1f;
                b.acceleration.x += ((float)rand() / float(RAND_MAX) - 0.5f) * jitterStrength;
                b.acceleration.y += ((float)rand() / RAND_MAX - 0.5f) * jitterStrength;
                b.acceleration.z += ((float)rand() / RAND_MAX - 0.5f) * jitterStrength;

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
    bool needsSplit(float splitRadius = 40.0f, float directionVarThresh = 0.5f) const
    {
        if (radius > splitRadius)
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
    std::vector<BoidUnit *> split(int numSplits = 2)
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
    std::vector<BoidUnit *> splitByClustering(int numClusters = 4)
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
    void splitInPlace()
    {
        if (!needsSplit())
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
    bool canMergeWith(const BoidUnit &other, float mergeDist = 60.0f, float velThresh = 0.5f, float maxRadius = 120.0f) const
    {
        if (center.distance(other.center) > mergeDist)
            return false;
        if ((averageVelocity - other.averageVelocity).length() > velThresh)
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
    void mergeWith(const BoidUnit &other)
    {
        boids.insert(boids.end(), other.boids.begin(), other.boids.end());
        computeBoundingSphere();
    }

    // 結合
    void mergeWith(BoidUnit *other, BoidUnit *parent)
    {
        boids.insert(boids.end(), other->boids.begin(), other->boids.end());
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
};

void printTree(const BoidUnit *node, int depth = 0)
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
};

class BoidTree
{
public:
    BoidUnit *root;
    int frameCount = 0;
    std::vector<BoidUnit *> leafCache;
    int splitIndex = 0;
    int mergeIndex = 0;

    BoidTree() : root(nullptr) {}
    // BoidTreeクラス内に追加
    static std::vector<Boid> generateRandomBoids(int count, float posRange, float velRange);
    void setFlockSize(int newSize, float posRange, float velRange);
    // BoidTreeクラス内に追加ここまで

    void build(std::vector<Boid> &boids, int maxPerUnit = 16, int level = 0)
    {
        root = new BoidUnit();
        root->level = level;
        buildRecursive(root, boids, maxPerUnit, level);
        printTree(root); // 木構造を出力
    }

    void buildRecursive(BoidUnit *node, std::vector<Boid> &boids, int maxPerUnit, int level)
    {
        if ((int)boids.size() <= maxPerUnit)
        {
            node->boids = boids;
            node->computeBoundingSphere();
            return;
        }
        int axis = 2;
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

    BoidUnit *findParent(BoidUnit *node, BoidUnit *target)
    {
        if (!node || node->children.empty())
            return nullptr;
        for (auto *c : node->children)
        {
            if (!c)
                continue; // 追加
            if (c == target)
                return node;
            BoidUnit *res = findParent(c, target);
            if (res)
                return res;
        }
        return nullptr;
    }

    void update(float dt = 1.0f)
    {
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

        if (!leafCache.empty())
        {
            // 分割
            for (int i = 0; i < 5 && splitIndex < (int)leafCache.size(); ++i, ++splitIndex)
            {
                BoidUnit *u = leafCache[splitIndex];
                if (u && u->needsSplit())
                {
                    u->splitInPlace();
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
    void trySplitRecursive(BoidUnit *node)
    {
        if (!node)
            return;
        if (node->isBoidUnit() && node->needsSplit())
        {
            node->splitInPlace();
        }
        for (auto *c : node->children)
            trySplitRecursive(c);
    }

    std::vector<Boid> getBoids() const
    {
        std::vector<Boid> result;
        collectBoids(root, result);
        return result;
    }

    void collectBoids(const BoidUnit *node, std::vector<Boid> &result) const
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

    void collectLeaves(const BoidUnit *node, std::vector<BoidUnit *> &leaves) const
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
                if (child) // 追加
                    collectLeaves(child, leaves);
            }
        }
    }
};
// JSオブジェクトからグローバルパラメータをセット
void setGlobalSpeciesParamsFromJS(val jsObj)
{
    std::cout << "cohesion=" << globalSpeciesParams.cohesion << std::endl;
    globalSpeciesParams.cohesion = jsObj["cohesion"].as<float>();
    globalSpeciesParams.separation = jsObj["separation"].as<float>();
    globalSpeciesParams.alignment = jsObj["alignment"].as<float>();
    globalSpeciesParams.maxSpeed = jsObj["maxSpeed"].as<float>();
    globalSpeciesParams.maxTurnAngle = jsObj["maxTurnAngle"].as<float>();
    globalSpeciesParams.separationRange = jsObj["separationRange"].as<float>();
    globalSpeciesParams.alignmentRange = jsObj["alignmentRange"].as<float>();
    globalSpeciesParams.cohesionRange = jsObj["cohesionRange"].as<float>();
}

std::vector<Boid> BoidTree::generateRandomBoids(int count, float posRange, float velRange)
{
    std::vector<Boid> boids;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int i = 0; i < count; ++i)
    {
        Boid b;
        b.position = Vec3(posDist(gen), posDist(gen), posDist(gen));
        b.velocity = Vec3(velDist(gen), velDist(gen), velDist(gen));
        b.acceleration = Vec3(0, 0, 0);
        b.id = i;
        b.stress = 0.0f;
        b.speciesId = 0;
        boids.push_back(b);
    }
    return boids;
}

void BoidTree::setFlockSize(int newSize, float posRange, float velRange)
{
    std::vector<Boid> currentBoids = getBoids();
    int current = static_cast<int>(currentBoids.size());

    if (newSize < current)
    {
        // ランダムに減らす
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(currentBoids.begin(), currentBoids.end(), gen);
        currentBoids.resize(newSize);
    }
    else if (newSize > current)
    {
        // ランダムに追加
        auto added = BoidTree::generateRandomBoids(newSize - current, posRange, velRange);
        currentBoids.insert(currentBoids.end(), added.begin(), added.end());
    }
    build(currentBoids, 32, 0);
}

EMSCRIPTEN_BINDINGS(my_module)
{
    value_object<Vec3>("Vec3")
        .field("x", &Vec3::x)
        .field("y", &Vec3::y)
        .field("z", &Vec3::z);

    value_object<SpeciesParams>("SpeciesParams")
        .field("cohesion", &SpeciesParams::cohesion)
        .field("separation", &SpeciesParams::separation)
        .field("alignment", &SpeciesParams::alignment)
        .field("maxSpeed", &SpeciesParams::maxSpeed)
        .field("minSpeed", &SpeciesParams::minSpeed)
        .field("maxTurnAngle", &SpeciesParams::maxTurnAngle)
        .field("separationRange", &SpeciesParams::separationRange)
        .field("alignmentRange", &SpeciesParams::alignmentRange)
        .field("cohesionRange", &SpeciesParams::cohesionRange);

    class_<Boid>("Boid")
        .constructor<>()
        .property("position", &Boid::position)
        .property("velocity", &Boid::velocity)
        .property("acceleration", &Boid::acceleration)
        .property("id", &Boid::id)
        .property("stress", &Boid::stress)
        .property("speciesId", &Boid::speciesId);

    class_<BoidTree>("BoidTree")
        .constructor<>()
        .function("build", &BoidTree::build)
        .function("update", &BoidTree::update)
        .function("getBoids", &BoidTree::getBoids)
        .function("setFlockSize", &BoidTree::setFlockSize)
        .class_function("generateRandomBoids", &BoidTree::generateRandomBoids)
        .property("root", &BoidTree::root, allow_raw_pointers());

    class_<BoidUnit>("BoidUnit")
        .property("center", &BoidUnit::center)
        .property("radius", &BoidUnit::radius)
        .property("children", &BoidUnit::children, allow_raw_pointers());

    register_vector<Boid>("VectorBoid");
    register_vector<BoidUnit *>("VectorBoidUnit");

    function("getGlobalSpeciesParams", &getGlobalSpeciesParams);
    function("setGlobalSpeciesParams", &setGlobalSpeciesParams);
    function("setGlobalSpeciesParamsFromJS", &setGlobalSpeciesParamsFromJS);
}