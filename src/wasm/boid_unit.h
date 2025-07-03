#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <vector>
#include <bitset>
#include <unordered_map>
#include <array>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "boid.h"
#include "species_params.h"

struct SoABuffers;

class BoidUnit
{
public:
    static int nextId; // 次の ID を管理する静的変数
    int id;            // 各 BoidUnit のユニークな ID
    BoidUnit* parent = nullptr; // 親ノードへのポインタ
    BoidUnit* topParent = nullptr; // 親ノードへのポインタ
    static constexpr int MAX_BOIDS = 16;     // Boid数の上限（local index）
    int speciesId = -1;
    SoABuffers *buf = nullptr;
    std::vector<int> indices;

    std::vector<float> cohesionMemories{};  // dt累積（-1.0fで未使用）
    std::bitset<MAX_BOIDS> activeNeighbors{};         // 使用中slotのインデックス

    std::vector<BoidUnit *> children;
    glm::vec3 center{}, averageVelocity{};
    float radius = 0.0f;
    int level = 0;
    int frameCount = 0;

    BoidUnit() : id(nextId++) {
        cohesionMemories.resize(MAX_BOIDS, 0.0f);
    }

    int getMaxID() const;
    bool isBoidUnit() const;
    void computeBoundingSphere();
    void computeBoidInteraction(float dt);
    void applyInterUnitInfluence(BoidUnit *other, float dt = 1.0f);
    void updateRecursive(float dt = 1.0f);
    bool needsSplit(float splitRadius = 40.0f, float directionVarThresh = 0.5f, int maxBoids = 64) const;
    std::vector<BoidUnit *> split(int numSplits = 2);
    std::vector<BoidUnit *> splitByClustering(int numClusters = 4);
    void splitInPlace(int maxBoids = 64);
    bool canMergeWith(const BoidUnit &other, float mergeDist = 60.0f, float velThresh = 0.5f, float maxRadius = 120.0f, int maxBoids = 32) const;
    void mergeWith(const BoidUnit &other);
    void mergeWith(BoidUnit *other, BoidUnit *parent);
    void addRepulsionToAllBoids(BoidUnit *unit, const glm::vec3 &repulsion);
    glm::vec3 fixRoll(const glm::vec3 &direction);
    static glm::quat dirToQuatRollZero(const glm::vec3 &forward);
    void applyPredatorInfluence(int gIdx, glm::vec3 &acceleration, const glm::vec3 &position);
    static float easeOut(float t);
};

void printTree(const BoidUnit *node, int depth = 0);