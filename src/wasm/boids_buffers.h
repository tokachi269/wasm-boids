#include "boid.h"
#include <bitset>
#include <boost/align/aligned_allocator.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>
#include <vector>

template <typename T> using A16 = boost::alignment::aligned_allocator<T, 16>;

// ダブルバッファ方式のSoA構造体
// positions/velocities/orientationsが読み取り用、*Writeが書き込み用
// 更新後にswapReadWrite()で入れ替えることで同期問題を回避
struct SoABuffers {
  std::vector<glm::vec3, A16<glm::vec3>> positions;
  std::vector<glm::vec3, A16<glm::vec3>> positionsWrite;
  std::vector<glm::vec3, A16<glm::vec3>> velocities;
  std::vector<glm::vec3, A16<glm::vec3>> velocitiesWrite;
  std::vector<glm::vec3, A16<glm::vec3>> accelerations;
  std::vector<glm::quat, A16<glm::quat>> orientations;
  std::vector<glm::quat, A16<glm::quat>> orientationsWrite;
  std::vector<glm::vec3, A16<glm::vec3>> predatorInfluences;
  std::vector<int> ids;
  std::vector<float> stresses;
  std::vector<int> speciesIds;
  std::vector<uint8_t> isAttracting; // 0: 吸引オフ, 1: 吸引オン
  std::vector<float> attractTimers;  // 吸引が続く残時間 (秒)

  std::vector<int> predatorTargetIndices; // 捕食者の捕食対象index
  std::vector<float>
      predatorTargetTimers; // 捕食者の捕食対象ターゲット残時間 (秒)

  std::unordered_map<int, std::unordered_map<int, float>> cohesionMemories;
  std::vector<float> predatorThreats; // 捕食圧の蓄積値（0-1）

  // 各BoidのcohesionMemoriesとactiveNeighbors（SOA形式）
  std::vector<std::vector<float>>
      boidCohesionMemories;                         // dt累積（-1.0fで未使用）
  std::vector<std::bitset<16>> boidActiveNeighbors; // 使用中slotのインデックス

  // 種族ごとの中心座標キャッシュ（フレームごとに1回計算）
  std::vector<glm::vec3> speciesCenters;
  std::vector<int> speciesCounts;

  void reserveAll(std::size_t n) {
    positions.reserve(n);
    positionsWrite.reserve(n);
    velocities.reserve(n);
    velocitiesWrite.reserve(n);
    accelerations.reserve(n);
    ids.reserve(n);
    stresses.reserve(n);
    speciesIds.reserve(n);
    isAttracting.reserve(n);
    attractTimers.reserve(n);
    orientations.reserve(n);
    orientationsWrite.reserve(n);
    predatorTargetIndices.reserve(n);
    predatorTargetTimers.reserve(n);
    predatorInfluences.reserve(n);
    predatorThreats.reserve(n);
    boidCohesionMemories.reserve(n);
    boidActiveNeighbors.reserve(n);
    // speciesCentersは種族数分だけなので個別管理
  }

  // Boid 数に合わせてフラグをクリア/サイズ調整
  void resizeAll(std::size_t n) {
    positions.resize(n);
    positionsWrite.resize(n);
    velocities.resize(n);
    velocitiesWrite.resize(n);
    orientations.resize(n, glm::quat(1, 0, 0, 0));
    orientationsWrite.resize(n, glm::quat(1, 0, 0, 0));
    accelerations.resize(n);
    ids.resize(n);
    stresses.resize(n);
    speciesIds.resize(n);
    isAttracting.resize(n, 0);
    attractTimers.resize(n, 0.0f);
    predatorTargetIndices.resize(n, -1);
    predatorTargetTimers.resize(n, 0.0f);
    predatorInfluences.resize(n, glm::vec3(0.0f));
    predatorThreats.resize(n, 0.0f);
    boidCohesionMemories.resize(n);
    boidActiveNeighbors.resize(n);
    // speciesCentersは種族数に応じて動的に確保
  }

  // 書き込みバッファを読み取りバッファから同期（初期化時用）
  void syncWriteFromRead() {
    positionsWrite = positions;
    velocitiesWrite = velocities;
    orientationsWrite = orientations;
  }

  // 読み取り/書き込みバッファを入れ替え（更新完了後に呼ぶ）
  void swapReadWrite() {
    positions.swap(positionsWrite);
    velocities.swap(velocitiesWrite);
    orientations.swap(orientationsWrite);
  }
};