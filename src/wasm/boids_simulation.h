#pragma once
#include <stack>
#include <vector>
#include <cstdint>
#include "boid_unit.h"
#include "boid.h"
#include "species_params.h"
#include "boids_buffers.h"
#include "spatial_index.h"
#include "boid_tree_spatial_index.h"

// 群れシミュレーションの世界状態（更新ループ・SoAバッファ・デバッグ集計）を保持する。
// かつては BoidUnit ツリー実装まで抱えていたため BoidTree と呼んでいたが、
// SpatialIndex 実装を分離したことで実態は「シミュレーション本体」なので改名する。
class BoidSimulation : public SpatialIndex
{
public:
    static BoidSimulation& instance() {
        static BoidSimulation instance; // シングルトンインスタンス
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
    int maxBoidsPerUnit = 32;
    SoABuffers buf;              // 中央バッファに一本化
    struct SpeciesEnvelope {
        glm::vec3 center = glm::vec3(0.0f);
        float radius = 0.0f;
        float count = 0.0f;
    };
    struct SpeciesCluster {
        glm::vec3 center = glm::vec3(0.0f);
        glm::vec3 avgVelocity = glm::vec3(0.0f);
        glm::vec3 frameSumPosition = glm::vec3(0.0f);
        // 分散(広がり)推定用に二乗和も保持する。
        // E[x^2] - (E[x])^2 から RMS 半径を計算でき、中心が多少ズレても安定する。
        glm::vec3 frameSumPositionSq = glm::vec3(0.0f);
        // leaf 単位で集計すると中心だけでは内部の広がりが落ちるため、
        // leaf 半径も加算して「塊の実寸」を過小評価しにくくする。
        float frameSumLeafRadius = 0.0f;
        glm::vec3 frameSumVelocity = glm::vec3(0.0f);
        float radius = 1.0f;
        float weight = 0.0f;
        int frameContributionCount = 0;
        int lastUpdateFrame = -1024;
        bool active = false;
    };

    // 小クラスター（speciesClusters）から更に上位の「群れ（大クラスター）」を推定する。
    // - “密集している小クラスター群”を連結成分としてまとめる
    // - 中心が毎フレーム飛ばないように、10秒スケールのEMAで追跡する
    // 1要素は 1つの群れ（大クラスター）を表す。
    struct SpeciesSchoolCluster {
        glm::vec3 center = glm::vec3(0.0f);
        glm::vec3 avgVelocity = glm::vec3(0.0f);
        float radius = 1.0f;
        float weight = 0.0f;
        int lastUpdateFrame = -1024;
        bool active = false;
    };
    std::vector<float> unitSimpleDensities;
    std::vector<SpeciesEnvelope> speciesEnvelopes;
    std::vector<std::vector<SpeciesCluster>> speciesClusters;
    std::vector<std::vector<SpeciesSchoolCluster>> speciesSchoolClusters;
    // JS 側に渡すため center.xyz + radius + count を 5 要素で詰めたフラット配列
    std::vector<float> speciesEnvelopeBuffer;

    // JS 側で「複数クラスター」をデバッグ可視化するためのフラット配列。
    // 1クラスターあたり 6 float: speciesId, center.x, center.y, center.z, radius, weight
    // - speciesId は JS 側で色分けに使う
    // - weight は「どれくらい生きているクラスターか」の目安（明度などに使える）
    std::vector<float> speciesClusterBuffer;
    bool speciesClusterBufferDirty = true;

    // JS 側で「大クラスター（群れ）」をデバッグ可視化するためのフラット配列。
    // 1クラスターあたり 6 float: speciesId, center.x, center.y, center.z, radius, weight
    // 形式を小クラスターと合わせることで、JS 側の描画ロジックを再利用しやすくする。
    std::vector<float> speciesSchoolClusterBuffer;
    bool speciesSchoolClusterBufferDirty = true;

    // BoidUnit プール
    std::stack<BoidUnit*> unitPool;

    // 現在使用中の SpatialIndex 実装。
    // - 既定は BoidUnit ツリーを使う BoidTreeSpatialIndex。
    // - 将来的に UniformGrid/BVH などへ差し替えるための注入ポイント。
    BoidTreeSpatialIndex treeSpatialIndex_;
    const SpatialIndex *activeSpatialIndex_ = &treeSpatialIndex_;

    // クラスター更新を間引く際の dt 蓄積（時間スケールのEMAを保つ）
    float clusterUpdateDtAccumulator_ = 0.0f;
    
    BoidSimulation();
    ~BoidSimulation();

    // ---- 参照専用の軽量アクセサ（外部からの直アクセスを減らす） ----
    int getFrameCount() const { return frameCount; }
    int getMaxBoidsPerUnit() const { return maxBoidsPerUnit; }
    const SoABuffers& getBuffers() const { return buf; }

    // 空間インデックス（現状は BoidUnit ツリー）を保持値で再構築する。
    void rebuildSpatialIndex() { build(); }

    // JS 側の色分け等に使う speciesIds 配列。
    uintptr_t getSpeciesIdsPtr() const {
        const auto &ids = buf.speciesIds;
        return ids.empty() ? 0 : reinterpret_cast<uintptr_t>(ids.data());
    }

    // SoA ダブルバッファの読み取り側→書き込み側を同期する（デバッグ/可視化用途）。
    void syncWriteFromReadBuffers() { buf.syncWriteFromRead(); }
    
    void setFlockSize(int newSize, float posRange, float velRange);
    void initializeBoids(const std::vector<SpeciesParams> &speciesParamsList, float posRange, float velRange);
    // 空間インデックスを再構築する（maxBoidsPerUnit を使用）。
    void build();
    void update(float dt = 1.0f);
    // バッファ更新
    uintptr_t getPositionsPtr();
    uintptr_t getVelocitiesPtr();
    uintptr_t getOrientationsPtr();
    int getBoidCount() const;
    std::unordered_map<int, int> collectBoidUnitMapping();
    void setUnitSimpleDensity(int unitId, float value);
    uintptr_t getUnitSimpleDensityPtr();
    int getUnitSimpleDensityCount() const;
    const std::vector<SpeciesSchoolCluster> *getSpeciesSchoolClusters(int speciesId) const;
    uintptr_t getSpeciesEnvelopePtr();
    int getSpeciesEnvelopeCount() const;

    // Species cluster debug export
    uintptr_t getSpeciesClustersPtr();
    int getSpeciesClustersCount() const;

    // Species school clusters debug export
    uintptr_t getSpeciesSchoolClustersPtr();
    int getSpeciesSchoolClustersCount() const;
    SpeciesParams getGlobalSpeciesParams(std::string species);
    void setGlobalSpeciesParams(const SpeciesParams &params);

    /**
     * 捕食者影響（警戒距離）の上限を返す。
     * - 捕食者の探索/警戒は「全非捕食者の predatorAlertRadius の最大値」を使う。
     * - これを毎回スキャンすると非常に重いので、setGlobalSpeciesParams() でキャッシュする。
     */
    float getMaxPredatorAlertRadius() const { return maxPredatorAlertRadius_; }

    // SpatialIndex implementation
    void forEachLeaf(const LeafVisitor &visitor) const override;
    void forEachLeafIntersectingSphere(const glm::vec3 &center, float radius,
                                       const LeafVisitor &visitor) const override;

    /**
     * 球交差クエリ（早期終了対応版）。
     *
     * - 外部近傍補完など「必要数が集まれば十分」な用途では、
     *   全葉を走査し続けるのは無駄になりやすい。
     * - visitor が false を返した時点で探索を打ち切る。
     *
     * 注意:
     * - SpatialIndex の仮想インターフェースは互換性維持のため変更しない。
    * - BoidSimulation 固有の高速パスとして提供する。
     */
    template <typename CancelableVisitor>
    void forEachLeafIntersectingSphereCancelable(const glm::vec3 &center, float radius,
                                                 CancelableVisitor &&visitor) const {
        // BoidUnit ツリーが有効な場合は tree 実装の cancelable を使う。
        // 将来的に別の SpatialIndex 実装へ差し替えた場合は、最後まで走査するフォールバックになる。
        if (activeSpatialIndex_ == &treeSpatialIndex_) {
            treeSpatialIndex_.forEachLeafIntersectingSphereCancelable(center, radius,
                std::forward<CancelableVisitor>(visitor));
            return;
        }

        // フォールバック: cancelできない場合は最後まで走査する。
        forEachLeafIntersectingSphere(center, radius, [&](const SpatialLeaf &leaf) {
            (void)visitor(leaf);
        });
    }

private:
    // ---- 内部実装（外部から呼ばれないので private に寄せる） ----
    // プール管理
    BoidUnit* getUnitFromPool();
    void returnUnitToPool(BoidUnit* unit);
    void clearPool();

    void initializeBoidMemories(const std::vector<SpeciesParams> &speciesParamsList);
    void buildRecursive(BoidUnit *node, const std::vector<int> &indices);
    void trySplitRecursive(BoidUnit *node);
    void collectLeaves(const BoidUnit *node, std::vector<BoidUnit *> &leaves) const;

    void updateSpeciesEnvelopes();
    void updateSpeciesClusters(float dt);
    void updateSpeciesSchoolClusters(float dt);
    const SpeciesEnvelope *getSpeciesEnvelope(int speciesId) const;
    const std::vector<SpeciesCluster> *getSpeciesClusters(int speciesId) const;

    enum DebugRequestBits : uint8_t {
        kDebugRequestSpeciesEnvelopes = 1 << 0,
        kDebugRequestSpeciesClusters = 1 << 1,
        kDebugRequestSpeciesSchoolClusters = 1 << 2,
    };

    // 可視化データの更新は「要求があった間だけ」有効化して、普段のコストを削る。
    uint8_t debugRequestBits_ = 0;
    int debugLastRequestFrame_ = -1000000;

    void markDebugRequest(uint8_t bits);

    void returnNodeToPool(BoidUnit* node);
    void collectLeavesForCache(BoidUnit *node, BoidUnit *parent);
    // レンダリング用ポインタを読み取りバッファに設定（描画時に使用）
    void setRenderPointersToReadBuffers();
    // レンダリング用ポインタを書き込みバッファに設定（デバッグ用）
    void setRenderPointersToWriteBuffers();

    // JavaScriptに公開するレンダリング用ポインタ（常に安定した読み取りバッファを指す）
    uintptr_t renderPositionsPtr_ = 0;
    uintptr_t renderVelocitiesPtr_ = 0;
    uintptr_t renderOrientationsPtr_ = 0;

    // speciesClusterBuffer の再構築（必要になった時だけ行う）
    void rebuildSpeciesClusterDebugBuffer();

    // speciesSchoolClusterBuffer の再構築（必要になった時だけ行う）
    void rebuildSpeciesSchoolClusterDebugBuffer();

    // 非捕食者種の predatorAlertRadius の最大値（捕食者影響範囲の上限）
    float maxPredatorAlertRadius_ = 1.0f;
};

extern std::vector<SpeciesParams> globalSpeciesParams;
