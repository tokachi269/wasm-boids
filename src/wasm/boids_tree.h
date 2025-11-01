#pragma once
#include <stack>
#include <vector>
#include <cstdint>
#include "boid_unit.h"
#include "boid.h"
#include "species_params.h"
#include "boids_buffers.h"
#include "spatial_index.h"

class BoidTree : public SpatialIndex
{
public:
    static BoidTree& instance() {
        static BoidTree instance; // シングルトンインスタンス
        return instance;
    }
    struct LeafCacheEntry {
        BoidUnit *node;
        BoidUnit *parent;
    };

    BoidUnit *root;
    int frameCount = 0;
    std::vector<LeafCacheEntry> leafCache;
    int splitIndex = 0;
    int mergeIndex = 0;
    int maxBoidsPerUnit = 16;
    SoABuffers buf;              // 中央バッファに一本化

    // BoidUnit プール
    std::stack<BoidUnit*> unitPool;
    
    BoidTree();
    ~BoidTree();
    
    // プール管理
    BoidUnit* getUnitFromPool();
    void returnUnitToPool(BoidUnit* unit);
    void clearPool();
    
    void setFlockSize(int newSize, float posRange, float velRange);
    void initializeBoids(const std::vector<SpeciesParams> &speciesParamsList, float posRange, float velRange);
    void initializeBoidMemories(const std::vector<SpeciesParams> &speciesParamsList);
    void build(int maxPerUnit = 16);
    void buildRecursive(BoidUnit *node, const std::vector<int> &indices, int maxPerUnit);
    void update(float dt = 1.0f);
    void trySplitRecursive(BoidUnit *node);
    // バッファ更新
    uintptr_t getPositionsPtr();
    uintptr_t getVelocitiesPtr();
    uintptr_t getOrientationsPtr();
    int getBoidCount() const;
    void collectLeaves(const BoidUnit *node, std::vector<BoidUnit *> &leaves) const;
    std::unordered_map<int, int> collectBoidUnitMapping();
    SpeciesParams getGlobalSpeciesParams(std::string species);
    void setGlobalSpeciesParams(const SpeciesParams &params);
    void rebuildTreeWithUnits(BoidUnit *node,
                              const std::vector<BoidUnit *> &units,
                              int maxPerUnit);

    // SpatialIndex implementation
    void forEachLeaf(const LeafVisitor &visitor) const override;
    void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                       const LeafVisitor &visitor) const override;

private:
    void returnNodeToPool(BoidUnit* node);
    void collectLeavesForCache(BoidUnit *node, BoidUnit *parent);
    void forEachLeafRecursive(const BoidUnit *node, const LeafVisitor &visitor) const;
    void forEachLeafIntersectingSphereRecursive(const BoidUnit *node,
                                                const glm::vec3 &center,
                                                float radius,
                                                const LeafVisitor &visitor) const;
    // レンダリング用ポインタを読み取りバッファに設定（描画時に使用）
    void setRenderPointersToReadBuffers();
    // レンダリング用ポインタを書き込みバッファに設定（デバッグ用）
    void setRenderPointersToWriteBuffers();

    // JavaScriptに公開するレンダリング用ポインタ（常に安定した読み取りバッファを指す）
    uintptr_t renderPositionsPtr_ = 0;
    uintptr_t renderVelocitiesPtr_ = 0;
    uintptr_t renderOrientationsPtr_ = 0;
};

extern std::vector<SpeciesParams> globalSpeciesParams;
