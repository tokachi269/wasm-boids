// Boids Tree：Boids特有のルールに注釈コメント追加

#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <emscripten/bind.h>

using namespace emscripten;

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    float distance(const Vec3& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator/(float v) const { return Vec3(x / v, y / v, z / v); }
    Vec3 operator*(float v) const { return Vec3(x * v, y * v, z * v); }

    Vec3& operator+=(const Vec3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
};

struct Boid {
    Vec3 position, velocity, acceleration;
    int id;
    float stress = 0.0f;
};

struct BoidStats {
    Vec3 sumVelocity, sumPosition, separation;
    int count = 0;
};

class BoidUnit {
public:
    std::vector<Boid> boids;
    std::vector<BoidUnit*> children;
    Vec3 center, averageVelocity;
    float radius = 0.0f;
    int level = 0;

    bool isBoidUnit() const { return children.empty(); }

    void computeBoundingSphere() {
        if (isBoidUnit()) {
            if (boids.empty()) return;
            center = Vec3();
            for (const auto& b : boids) center = center + b.position;
            center = center / boids.size();
            radius = 0.0f;
            for (const auto& b : boids) {
                float d = center.distance(b.position);
                if (d > radius) radius = d;
            }
        } else {
            center = Vec3();
            for (const auto& c : children) center = center + c->center;
            center = center / children.size();
            radius = 0.0f;
            for (const auto& c : children) {
                float d = center.distance(c->center) + c->radius;
                if (d > radius) radius = d;
            }
        }
    }

    BoidStats computeBoidStats(Boid& self, const std::vector<Boid>& others) const {
        BoidStats s;
        for (const auto& n : others) {
            if (n.id == self.id) continue;
            s.sumVelocity = s.sumVelocity + n.velocity;
            s.sumPosition = s.sumPosition + n.position;
            Vec3 diff = self.position - n.position;
            float dist = self.position.distance(n.position);
            if (dist > 0.01f) {
                s.separation = s.separation + (diff / (dist * dist));
                self.stress += (n.stress - self.stress) * 0.1f;
            }
            s.count++;
        }
        return s;
    }

    void applyInterUnitInfluence(BoidUnit* other) {
        for (Boid& a : boids) {
            for (const Boid& b : other->boids) {
                float dist = a.position.distance(b.position);
                if (dist < 500.0f) {
                    Vec3 toB = b.position - a.position;
                    a.acceleration += toB * 0.1f;
                }
            }
        }
    }

    void updateRecursive(float dt = 1.0f) {
        if (isBoidUnit()) {
            for (auto& b : boids) {
                BoidStats s = computeBoidStats(b, boids);
                Vec3 align, cohes, separ;
                if (s.count > 0) {
                    align = (s.sumVelocity / s.count - b.velocity) * 0.05f;
                    cohes = ((s.sumPosition / s.count) - b.position) * 0.01f * (1.0f - b.stress);
                    separ = s.separation * (0.1f + b.stress * 0.4f);
                }
                float boost = 1.0f + b.stress * 0.8f;
                b.acceleration = (align + cohes + separ) * boost;
                b.velocity += b.acceleration * dt;
                b.position += b.velocity * dt;
                b.stress = std::max(0.0f, b.stress - 0.005f);
            }
            center = averageVelocity = Vec3();
            for (const auto& b : boids) {
                center = center + b.position;
                averageVelocity = averageVelocity + b.velocity;
            }
            if (!boids.empty()) {
                center = center / boids.size();
                averageVelocity = averageVelocity / boids.size();
            }
        } else {
            for (auto& c : children) c->updateRecursive(dt);
            // inter-unit influence
            for (size_t i = 0; i < children.size(); ++i) {
                for (size_t j = i + 1; j < children.size(); ++j) {
                    children[i]->applyInterUnitInfluence(children[j]);
                    children[j]->applyInterUnitInfluence(children[i]);
                }
            }
            center = averageVelocity = Vec3();
            for (const auto& c : children) {
                center = center + c->center;
                averageVelocity = averageVelocity + c->averageVelocity;
            }
            if (!children.empty()) {
                center = center / children.size();
                averageVelocity = averageVelocity / children.size();
            }
        }
        computeBoundingSphere();
    }
};

class BoidTree {
public:
    BoidUnit* root;
    BoidTree() : root(nullptr) {}

    void build(std::vector<Boid>& boids, int maxPerUnit = 16, int level = 0) {
        root = new BoidUnit();
        root->level = level;
        buildRecursive(root, boids, maxPerUnit, level);
    }

    void buildRecursive(BoidUnit* node, std::vector<Boid>& boids, int maxPerUnit, int level) {
        if ((int)boids.size() <= maxPerUnit) {
            node->boids = boids;
            node->computeBoundingSphere();
            return;
        }
        int axis = level % 3;
        std::sort(boids.begin(), boids.end(), [axis](const Boid& a, const Boid& b) {
            if (axis == 0) return a.position.x < b.position.x;
            if (axis == 1) return a.position.y < b.position.y;
            return a.position.z < b.position.z;
        });
        size_t mid = boids.size() / 2;
        std::vector<Boid> left(boids.begin(), boids.begin() + mid);
        std::vector<Boid> right(boids.begin() + mid, boids.end());
        BoidUnit* leftChild = new BoidUnit();
        BoidUnit* rightChild = new BoidUnit();
        leftChild->level = rightChild->level = level + 1;
        buildRecursive(leftChild, left, maxPerUnit, level + 1);
        buildRecursive(rightChild, right, maxPerUnit, level + 1);
        node->children = { leftChild, rightChild };
        node->computeBoundingSphere();
    }

    void update(float dt = 1.0f) {
        if (root) root->updateRecursive(dt);
    }

    std::vector<Boid> getBoids() const {
        std::vector<Boid> result;
        collectBoids(root, result);
        return result;
    }

    void collectBoids(const BoidUnit* node, std::vector<Boid>& result) const {
        if (!node) return;
        if (node->isBoidUnit()) {
            result.insert(result.end(), node->boids.begin(), node->boids.end());
        } else {
            for (const auto* child : node->children) {
                collectBoids(child, result);
            }
        }
    }
};

EMSCRIPTEN_BINDINGS(my_module) {
    value_object<Vec3>("Vec3")
        .field("x", &Vec3::x)
        .field("y", &Vec3::y)
        .field("z", &Vec3::z);

    class_<Boid>("Boid")
        .constructor<>()
        .property("position", &Boid::position)
        .property("velocity", &Boid::velocity)
        .property("acceleration", &Boid::acceleration)
        .property("id", &Boid::id)
        .property("stress", &Boid::stress);

    class_<BoidTree>("BoidTree")
        .constructor<>()
        .function("build", &BoidTree::build)
        .function("update", &BoidTree::update)
        .function("getBoids", &BoidTree::getBoids);

    register_vector<Boid>("VectorBoid");
}
