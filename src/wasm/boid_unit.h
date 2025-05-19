#pragma once
#include <vector>
#include "vec3.h"
#include "boid.h"
#include "species_params.h"

class BoidUnit {
public:
    std::vector<Boid> boids;
    std::vector<BoidUnit *> children;
    Vec3 center, averageVelocity;
    float radius = 0.0f;
    int level = 0;

    bool isBoidUnit() const;
    void computeBoundingSphere();
    BoidStats computeBoidStats(Boid &self, const std::vector<Boid> &others) const;
    void applyInterUnitInfluence(BoidUnit *other);
    void updateRecursive(float dt = 1.0f);
    bool needsSplit(float splitRadius = 40.0f, float directionVarThresh = 0.5f, int maxBoids = 64) const;
    std::vector<BoidUnit *> split(int numSplits = 2);
    std::vector<BoidUnit *> splitByClustering(int numClusters = 4);
    void splitInPlace(int maxBoids = 64);
    bool canMergeWith(const BoidUnit &other, float mergeDist = 60.0f, float velThresh = 0.5f, float maxRadius = 120.0f, int maxBoids = 32) const;
    void mergeWith(const BoidUnit &other);
    void mergeWith(BoidUnit *other, BoidUnit *parent);
};

void printTree(const BoidUnit *node, int depth = 0);