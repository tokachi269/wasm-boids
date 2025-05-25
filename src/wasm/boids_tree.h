#pragma once
#include <vector>
#include "boid_unit.h"
#include "boid.h"
#include "species_params.h"

class BoidTree {
public:
    BoidUnit *root;
    int frameCount = 0;
    std::vector<BoidUnit *> leafCache;
    int splitIndex = 0;
    int mergeIndex = 0;
    int maxBoidsPerUnit = 32;
    std::vector<float> positionBuffer;
    std::vector<float> velocityBuffer;

    BoidTree();
    static std::vector<Boid> generateRandomBoids(int count, float posRange, float velRange);
    void setFlockSize(int newSize, float posRange, float velRange);
    void build(std::vector<Boid> &boids, int maxPerUnit = 16, int level = 0);
    void buildRecursive(BoidUnit *node, std::vector<Boid> &boids, int maxPerUnit, int level);
    BoidUnit *findParent(BoidUnit *node, BoidUnit *target);
    void update(float dt = 1.0f);
    void trySplitRecursive(BoidUnit *node);
    std::vector<Boid> getBoids() const;
    void collectBoids(const BoidUnit *node, std::vector<Boid> &result) const;
    void collectLeaves(const BoidUnit *node, std::vector<BoidUnit *> &leaves) const;

    // バッファ更新
    void updatePositionBuffer();
    void updateVelocityBuffer();
    uintptr_t getPositionBufferPtr();
    uintptr_t getVelocityBufferPtr();
    int getBoidCount() const;
};

extern SpeciesParams globalSpeciesParams;
SpeciesParams getGlobalSpeciesParams();
void setGlobalSpeciesParams(const SpeciesParams &params);