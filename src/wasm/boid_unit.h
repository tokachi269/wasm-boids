#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <vector>
#include <bitset>
#include "vec3.h"
#include "boid.h"
#include "species_params.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
struct SoABuffers;

const int MAX_BOIDS = 10000; // Define a maximum number of boids

class BoidUnit
{
public:
    SoABuffers *buf = nullptr;
    std::vector<int> indices;
    std::vector<float> cohesionMemories; // Replace unordered_map with vector
    std::vector<int> idToIndex;          // ID-to-index mapping table
    std::bitset<MAX_BOIDS> visibleIds;   // Replace unordered_set with bitset

    std::vector<BoidUnit *> children;
    glm::vec3 center, averageVelocity;
    float radius = 0.0f;
    int level = 0;
    glm::vec3 influence; // 追加: 他ユニットからの影響を蓄積
    int frameCount = 0;

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

    void initializeBuffers(int maxBoids) {
        cohesionMemories.resize(maxBoids, 0.0f);
        idToIndex.resize(maxBoids, -1); // -1 indicates invalid index
    }
};

void printTree(const BoidUnit *node, int depth = 0);