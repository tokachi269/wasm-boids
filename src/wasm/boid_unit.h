#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <vector>
#include "vec3.h"
#include "boid.h"
#include "species_params.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>

class BoidUnit
{
public:
    // SOA形式のデータ
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> velocities;
    std::vector<glm::vec3> accelerations;
    std::vector<int> ids;
    std::vector<float> stresses;
    std::vector<int> speciesIds;
    std::unordered_map<int, std::unordered_map<int, float>> cohesionMemories;

    std::vector<BoidUnit *> children;
    glm::vec3 center, averageVelocity;
    float radius = 0.0f;
    int level = 0;
    glm::vec3 influence; // 追加: 他ユニットからの影響を蓄積
    int frameCount = 0;

    bool isBoidUnit() const;
    void computeBoundingSphere();
    void computeBoidInteraction(size_t index, float dt);
    void applyInterUnitInfluence(BoidUnit *other);
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