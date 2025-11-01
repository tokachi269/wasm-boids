#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <vector>
#include <cstddef>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

struct SoABuffers;
class LbvhIndex;

class BoidUnit
{
public:
    static int nextId; // 次の ID を管理する静的変数
    int id;            // 各 BoidUnit のユニークな ID
    SoABuffers *buf = nullptr;

    std::vector<int> indices;        // このユニットが保持する Boid のグローバルインデックス
    std::vector<BoidUnit *> children; // KD木時代の互換用。LBVH移行後は空のまま使用。
    glm::vec3 center{}, averageVelocity{};
    float radius = 0.0f;
    int frameCount = 0;
    int speciesId = -1;

    BoidUnit();
    ~BoidUnit() = default;

    bool isBoidUnit() const; // 子を持たない場合に葉扱い
    bool needsSplit(float radiusThreshold, float densityThreshold,
                    int maxPerUnit) const;
    void splitInPlace(int maxPerUnit);
    bool canMergeWith(const BoidUnit &other) const;
    void mergeWith(BoidUnit *other);

    void computeBoundingSphere();
    static glm::quat dirToQuatRollZero(const glm::vec3 &forward);
    static float easeOut(float t);
};

void computeBoidInteractionsRange(SoABuffers &buf, const LbvhIndex &index,
                                  int begin, int end, float dt);

void updateBoidKinematicsRange(SoABuffers &buf, int begin, int end,
                               float dt);