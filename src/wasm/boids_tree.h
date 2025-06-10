#pragma once
#include <vector>
#include "boid_unit.h"
#include "boid.h"
#include "species_params.h"
#include "boids_buffers.h"

class BoidTree
{
public:
    static BoidTree& instance() {
        static BoidTree instance; // シングルトンインスタンス
        return instance;
    }
    BoidUnit *root;
    int frameCount = 0;
    std::vector<BoidUnit *> leafCache;
    int splitIndex = 0;
    int mergeIndex = 0;
    int maxBoidsPerUnit = 32;
    SoABuffers buf;              // 中央バッファに一本化

    BoidTree();
    void setFlockSize(int newSize, float posRange, float velRange);
    void initializeBoids(const std::vector<SpeciesParams> &speciesParamsList, float posRange, float velRange);
    void build(int maxPerUnit = 16, int level = 0);
    void buildRecursive(BoidUnit *node, const std::vector<int> &indices, int maxPerUnit, int level);
    BoidUnit *findParent(BoidUnit *node, BoidUnit *target);
    void update(float dt = 1.0f);
    void trySplitRecursive(BoidUnit *node);
    // バッファ更新
    uintptr_t getPositionsPtr();
    uintptr_t getVelocitiesPtr();
    uintptr_t getOrientationsPtr();
    int getBoidCount() const;
    void collectLeaves(const BoidUnit *node, std::vector<BoidUnit *> &leaves) const;

    SpeciesParams getGlobalSpeciesParams(std::string species);
    void setGlobalSpeciesParams(const SpeciesParams &params);
};

extern std::vector<SpeciesParams> globalSpeciesParams;
