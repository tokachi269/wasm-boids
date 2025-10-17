#include <vector>
#include <bitset>
#include <unordered_map>
#include "boid.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <boost/align/aligned_allocator.hpp>
template <typename T>
using A16 = boost::alignment::aligned_allocator<T, 16>;

struct SoABuffers
{
    std::vector<glm::vec3, A16<glm::vec3>> positions;
    std::vector<glm::vec3, A16<glm::vec3>> velocities;
    std::vector<glm::vec3, A16<glm::vec3>> accelerations;
    std::vector<glm::quat, A16<glm::quat>> orientations;
    std::vector<glm::vec3, A16<glm::vec3>> predatorInfluences;
    std::vector<int> ids;
    std::vector<float> stresses;
    std::vector<int> speciesIds;
    std::vector<uint8_t> isAttracting; // 0: 吸引オフ, 1: 吸引オン
    std::vector<float> attractTimers;  // 吸引が続く残時間 (秒)

    std::vector<int> predatorTargetIndices;   // 捕食者の捕食対象index
    std::vector<float> predatorTargetTimers;  // 捕食者の捕食対象ターゲット残時間 (秒)

    std::unordered_map<int, std::unordered_map<int, float>> cohesionMemories;

    // 各BoidのcohesionMemoriesとactiveNeighbors（SOA形式）
    std::vector<std::vector<float>> boidCohesionMemories;  // dt累積（-1.0fで未使用）
    std::vector<std::bitset<16>> boidActiveNeighbors;      // 使用中slotのインデックス
    std::vector<std::vector<int>> boidNeighborSlots;       // slot -> neighbor boid index

    void reserveAll(std::size_t n)
    {
        positions.reserve(n);
        velocities.reserve(n);
        accelerations.reserve(n);
        ids.reserve(n);
        stresses.reserve(n);
        speciesIds.reserve(n);
        isAttracting.reserve(n);
        attractTimers.reserve(n);
        orientations.reserve(n);
        predatorTargetIndices.reserve(n);
        predatorTargetTimers.reserve(n);
        predatorInfluences.reserve(n);
        boidCohesionMemories.reserve(n);
        boidActiveNeighbors.reserve(n);
        boidNeighborSlots.reserve(n);
    }

    // Boid 数に合わせてフラグをクリア/サイズ調整
    void resizeAll(std::size_t n)
    {
        positions.resize(n);
        velocities.resize(n);
        orientations.resize(n, glm::quat(1, 0, 0, 0));
        accelerations.resize(n);
        ids.resize(n);
        stresses.resize(n);
        speciesIds.resize(n);
        isAttracting.resize(n, 0);
        attractTimers.resize(n, 0.0f);
        predatorTargetIndices.resize(n, -1);
        predatorTargetTimers.resize(n, 0.0f);
        predatorInfluences.resize(n, glm::vec3(0.0f));
        boidCohesionMemories.resize(n);
        boidActiveNeighbors.resize(n);
        boidNeighborSlots.resize(n);
    }
};