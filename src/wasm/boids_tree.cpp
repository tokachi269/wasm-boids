#include "vec3.h"
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <emscripten/bind.h>
#include <iostream>

using namespace emscripten;

struct SpeciesParams {
    float cohesion = 0.01f;
    float separation = 0.1f;
    float alignment = 0.05f;
};

// グローバル共通
SpeciesParams globalSpeciesParams;

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
                        float influence = std::max(0.0f, 1.0f - (dist / 40.0f)); // 40.0fは最大影響距離
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
                // std::cout << "[Unit-Unit] level=" << level
                //           << " <-> " << other->level
                //           << " | center=(" << center.x << "," << center.y << "," << center.z << ")"
                //           << " <-> (" << other->center.x << "," << other->center.y << "," << other->center.z << ")"
                //           << " | dist=" << dist << std::endl;

                Vec3 diff = center - other->center;
                Vec3 separation = diff / (dist * dist);
                Vec3 align = (other->averageVelocity - averageVelocity) * 1.0f;
                Vec3 cohes = (other->center - center) * 0.5f;

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

                float scale = std::sqrt((float)allBoids.size() * otherBoids.size()) / 10.0f;
                if (scale < 1.0f)
                    scale = 1.0f;

                // 影響を加算
                for (Boid *b : allBoids)
                {
                    b->acceleration += align * scale;
                    b->acceleration += cohes * scale;
                    b->acceleration += separation * 0.1f * scale;
                    // std::cout << "[Unit-Unit-Effect] id=" << b->id << " scale=" << scale << std::endl;
                }
                for (Boid *b : otherBoids)
                {
                    b->acceleration -= align * scale;
                    b->acceleration -= cohes * scale;
                    b->acceleration -= separation * 0.1f * scale;
                }
            }
        }
    }

    void updateRecursive(float dt = 1.0f)
    {
        if (isBoidUnit())
        {
            for (auto &b : boids)
            {
                BoidStats s = computeBoidStats(b, boids);
                Vec3 align, cohes, separ;
                if (s.count > 0)
                {
                    /** 整列 (Alignment): 他のBoidの平均速度に合わせる */
                    align = (s.sumVelocity / s.count - b.velocity) * globalSpeciesParams.alignment;

                    /** 結合 (Cohesion): 他のBoidの中心に向かう力 */
                    cohes = ((s.sumPosition / s.count) - b.position) * globalSpeciesParams.cohesion * (1.0f - b.stress);

                    /** 集合 (Separation): 他のBoidとの距離を保つ力 */
                    separ = s.separation * (globalSpeciesParams.separation + b.stress * 0.4f);
                }

                /** ストレスに応じた加速の調整 */
                float boost = 1.0f + b.stress * 0.8f;

                // 微小なランダム揺らぎ（5フレームに1回、Boidごとにずらす）
                int jitterSeed = rand();
                if ((jitterSeed + b.id) % 5 == 0)
                {
                    Vec3 jitter(
                        ((rand() % 2001) - 1000) / 1000.0f,
                        ((rand() % 2001) - 1000) / 1000.0f,
                        ((rand() % 2001) - 1000) / 1000.0f);
                    b.acceleration += jitter;
                }
                /** 速度の更新 */
                Vec3 desiredVelocity = b.velocity + b.acceleration * dt;
                float maxTurnAngle = 0.01f; // ラジアン/フレーム
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
                float maxSpeed = 1.0f;
                if (speed > maxSpeed)
                {
                    b.velocity = b.velocity * (maxSpeed / speed);
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

            /** 中心位置と平均速度の再計算 */
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
    bool needsSplit(float splitRadius = 80.0f, float directionVarThresh = 0.5f) const {
        if (radius > splitRadius) return true;
        // 方向のバラつき判定
        if (boids.size() > 1) {
            Vec3 avg = Vec3();
            for (const auto& b : boids) avg += b.velocity.normalized();
            avg = avg / boids.size();
            float var = 0.0f;
            for (const auto& b : boids)
                var += (b.velocity.normalized() - avg).length();
            var /= boids.size();
            if (var > directionVarThresh) return true;
        }
        return false;
    }

    // 2分割（例：最大分散軸で分割）
    std::vector<BoidUnit*> split() {
        int axis = 0;
        float maxVar = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float mean = 0, var = 0;
            for (const auto& b : boids)
                mean += (i==0?b.position.x:(i==1?b.position.y:b.position.z));
            mean /= boids.size();
            for (const auto& b : boids) {
                float v = (i==0?b.position.x:(i==1?b.position.y:b.position.z)) - mean;
                var += v*v;
            }
            if (var > maxVar) { maxVar = var; axis = i; }
        }
        std::vector<Boid> left, right;
        float mid = 0;
        for (const auto& b : boids)
            mid += (axis==0?b.position.x:(axis==1?b.position.y:b.position.z));
        mid /= boids.size();
        for (const auto& b : boids) {
            if ((axis==0?b.position.x:(axis==1?b.position.y:b.position.z)) < mid)
                left.push_back(b);
            else
                right.push_back(b);
        }
        BoidUnit* l = new BoidUnit(); l->boids = left; l->computeBoundingSphere();
        BoidUnit* r = new BoidUnit(); r->boids = right; r->computeBoundingSphere();
        return {l, r};
    }

    // 親ノードから自身を分割する
    void splitInPlace() {
        if (!needsSplit()) return;
        auto splits = split();
        this->boids.clear();
        this->children = splits;
        for (auto* c : children) c->level = this->level + 1;
        computeBoundingSphere();
        // 必要なら親ノードから再帰的に統計量を更新
    }

    // 結合可能か判定
    bool canMergeWith(const BoidUnit& other, float mergeDist = 60.0f, float velThresh = 0.5f, float maxRadius = 120.0f) const {
        if (center.distance(other.center) > mergeDist) return false;
        if ((averageVelocity - other.averageVelocity).length() > velThresh) return false;
        // 結合後の半径
        Vec3 newCenter = (center * boids.size() + other.center * other.boids.size()) / (boids.size() + other.boids.size());
        float newRadius = 0.0f;
        for (const auto& b : boids) newRadius = std::max(newRadius, newCenter.distance(b.position));
        for (const auto& b : other.boids) newRadius = std::max(newRadius, newCenter.distance(b.position));
        if (newRadius > maxRadius) return false;
        return true;
    }

    // 結合
    void mergeWith(const BoidUnit& other) {
        boids.insert(boids.end(), other.boids.begin(), other.boids.end());
        computeBoundingSphere();
    }

    // 結合
    void mergeWith(BoidUnit* other, BoidUnit* parent) {
        boids.insert(boids.end(), other->boids.begin(), other->boids.end());
        computeBoundingSphere();
        // 親ノードのchildrenからotherを除去
        if (parent) {
            auto it = std::find(parent->children.begin(), parent->children.end(), other);
            if (it != parent->children.end()) parent->children.erase(it);
        }
        // otherノードのdeleteも検討（メモリ管理注意）
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
    std::vector<BoidUnit*> leafCache;
    int splitIndex = 0;
    int mergeIndex = 0;

    BoidTree() : root(nullptr) {}

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

    BoidUnit* findParent(BoidUnit* node, BoidUnit* target) {
        if (!node || node->children.empty()) return nullptr;
        for (auto* c : node->children) {
            if (c == target) return node;
            BoidUnit* res = findParent(c, target);
            if (res) return res;
        }
        return nullptr;
    }

    void update(float dt = 1.0f) {
        if (root)
            root->updateRecursive(dt);

        frameCount++;
        if (frameCount % 10 == 0) {
            leafCache.clear();
            collectLeaves(root, leafCache);
            splitIndex = 0;
            mergeIndex = 0;
        }

        if (!leafCache.empty()) {
            // 分割
            for (int i = 0; i < 12 && splitIndex < (int)leafCache.size(); ++i, ++splitIndex) {
                BoidUnit* u = leafCache[splitIndex];
                if (u && u->needsSplit()) {
                    u->splitInPlace();
                    leafCache.clear();
                    collectLeaves(root, leafCache);
                    break;
                }
            }
            // 結合
            for (int i = 0; i < 12 && mergeIndex < (int)leafCache.size(); ++i, ++mergeIndex) {
                for (int j = mergeIndex + 1; j < (int)leafCache.size(); ++j) {
                    BoidUnit* a = leafCache[mergeIndex];
                    BoidUnit* b = leafCache[j];
                    if (a && b && a->canMergeWith(*b)) {
                        BoidUnit* parent = findParent(root, b);
                        if (parent) {
                            a->mergeWith(b, parent);
                            leafCache.clear();
                            collectLeaves(root, leafCache);
                        }
                        break;
                    }
                }
            }
        }
    }

    // 分割判定を局所的に適用
    void trySplitRecursive(BoidUnit* node) {
        if (!node) return;
        if (node->isBoidUnit() && node->needsSplit()) {
            node->splitInPlace();
        }
        for (auto* c : node->children) trySplitRecursive(c);
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

    void collectLeaves(const BoidUnit* node, std::vector<BoidUnit*>& leaves) const {
        if (!node) return;
        if (node->isBoidUnit()) {
            leaves.push_back(const_cast<BoidUnit*>(node));
        } else {
            for (const auto* child : node->children) {
                collectLeaves(child, leaves);
            }
        }
    }
};

EMSCRIPTEN_BINDINGS(my_module)
{
    value_object<Vec3>("Vec3")
        .field("x", &Vec3::x)
        .field("y", &Vec3::y)
        .field("z", &Vec3::z);

    value_object<SpeciesParams>("SpeciesParams")
        .field("cohesion", &SpeciesParams::cohesion)
        .field("separation", &SpeciesParams::separation)
        .field("alignment", &SpeciesParams::alignment);

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
        .function("getBoids", &BoidTree::getBoids);

    register_vector<Boid>("VectorBoid");
}
