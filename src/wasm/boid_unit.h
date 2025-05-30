#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <vector>
#include "vec3.h"
#include "boid.h"
#include "species_params.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
struct SoABuffers;

class BoidUnit
{
public:
    SoABuffers *buf = nullptr; // Not owning
    std::vector<int> indices;  // ← 既存移動元
    std::unordered_map<int, std::unordered_map<int, float>> cohesionMemories; // 修正

    std::vector<BoidUnit *> children;
    glm::vec3 center, averageVelocity;
    float radius = 0.0f;
    int level = 0;
    glm::vec3 influence; // 追加: 他ユニットからの影響を蓄積
    int frameCount = 0;

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
};

void printTree(const BoidUnit *node, int depth = 0);