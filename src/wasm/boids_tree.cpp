#include <vector>
#include <cmath>
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
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

struct Boid {
    Vec3 position;
    int id;
};

// 簡易的な2分木構造（KD-Treeに近い）
class BoidTreeNode {
public:
    Boid* boid;
    BoidTreeNode* left;
    BoidTreeNode* right;

    BoidTreeNode() : boid(nullptr), left(nullptr), right(nullptr) {}
};

class BoidTree {
public:
    BoidTreeNode* root;

    BoidTree() : root(nullptr) {}

    void build(std::vector<Boid>& boids, int depth = 0) {
        root = buildRecursive(boids, depth);
    }

private:
    BoidTreeNode* buildRecursive(std::vector<Boid>& boids, int depth) {
        if (boids.empty()) return nullptr;

        int axis = depth % 3;
        size_t mid = boids.size() / 2;

        std::sort(boids.begin(), boids.end(), [axis](const Boid& a, const Boid& b) {
            if (axis == 0) return a.position.x < b.position.x;
            if (axis == 1) return a.position.y < b.position.y;
            return a.position.z < b.position.z;
        });

        BoidTreeNode* node = new BoidTreeNode();
        node->boid = new Boid(boids[mid]); // ポインタを作成

        std::vector<Boid> leftBoids(boids.begin(), boids.begin() + mid);
        std::vector<Boid> rightBoids(boids.begin() + mid + 1, boids.end());

        node->left = buildRecursive(leftBoids, depth + 1);
        node->right = buildRecursive(rightBoids, depth + 1);

        return node;
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
        .property("id", &Boid::id);

    class_<BoidTree>("BoidTree")
        .constructor<>()
        .function("build", &BoidTree::build);

    register_vector<Boid>("VectorBoid"); // std::vector<Boid> を登録
}
