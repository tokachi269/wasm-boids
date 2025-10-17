#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>

#include "boids_buffers.h"
#include "dynamic_bvh.h"
#include "species_params.h"

class BoidTree
{
public:
    static BoidTree& instance() {
        static BoidTree instance;
        return instance;
    }

    int frameCount = 0;
    SoABuffers buf;

    DynamicBVH bvh;
    std::vector<int> proxyIds;
    float maxInteractionRadius = 10.0f;
    glm::vec3 fatMargin = glm::vec3(1.0f);

    BoidTree();
    ~BoidTree();
    
    void setFlockSize(int newSize, float posRange, float velRange);
    void initializeBoids(const std::vector<SpeciesParams> &speciesParamsList, float posRange, float velRange);
    void initializeBoidMemories(const std::vector<SpeciesParams> &speciesParamsList);
    void build(int maxPerUnit = 16, int level = 0);
    void update(float dt = 1.0f);
    // バッファ更新
    uintptr_t getPositionsPtr();
    uintptr_t getVelocitiesPtr();
    uintptr_t getOrientationsPtr();
    int getBoidCount() const;
    std::unordered_map<int, int> collectBoidUnitMapping();
    SpeciesParams getGlobalSpeciesParams(std::string species);
    void setGlobalSpeciesParams(const SpeciesParams &params);

private:
    void rebuildBVH();
    void ensureProxyStorage();
    void updateBVH();
    void processRange(int start, int end, float dt);
    void integrateRange(int start, int end, float dt);
    glm::vec3 computeFatMargin(int speciesId) const;
};

extern std::vector<SpeciesParams> globalSpeciesParams;
