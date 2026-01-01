#pragma once

#include "boid.h"
#include <array>
#include <bitset>
#include <boost/align/aligned_allocator.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

template <typename T> using A16 = boost::alignment::aligned_allocator<T, 16>;


// ダブルバッファ方式のSoA構造体
// positions/velocities/orientationsが読み取り用、*Writeが書き込み用
// 更新後にswapReadWrite()で入れ替えることで同期問題を回避
struct SoABuffers {
  // 近傍キャッシュは固定長のスロットで扱う。
  // 旧版は 32bit マスクで管理していたため、互換性と挙動安定性の観点で 32 を上限とする。
  // maxNeighbors がこれを超える場合は初期化時にクランプする。
  static constexpr std::size_t NeighborSlotCount = 32;

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
  std::vector<float> predatorRestTimers;  // 捕食者の休憩残時間 (秒)
  std::vector<float> predatorChaseTimers; // 現在の追跡経過時間 (秒)

  // 捕食者の簡易ステート補助ベクトル
  // - approach: 群れ（獲物密集）へ突っ込むための目標方向
  // - disengage: 追跡終了後に群れから離脱するための目標方向
  std::vector<glm::vec3, A16<glm::vec3>> predatorApproachDirs;
  std::vector<glm::vec3, A16<glm::vec3>> predatorDisengageDirs;

  std::vector<float> predatorThreats; // 捕食圧の蓄積値（0-1）

  // 各BoidのcohesionMemoriesとactiveNeighbors（SOA形式）
    // WebAssembly では unordered_map の破棄が深い再帰になりがちなので、近傍キャッシュは
    // フラットな vector で管理してスタック消費を抑える。
    std::vector<std::vector<float>>
      boidCohesionMemories;                         // dt累積（-1.0fで未使用）
  std::vector<std::bitset<NeighborSlotCount>>
      boidActiveNeighbors; // 使用中slotのインデックス
  // boidActiveNeighbors の各 slot に対応する「近傍Boidのグローバルindex」。
  // これにより BoidUnit(葉) の indices 順序に依存せず、KD-tree/SpatialIndex の結果を
  // 安定してキャッシュできる。
  std::vector<std::array<int, NeighborSlotCount>> boidNeighborIndices;

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
    predatorRestTimers.reserve(n);
    predatorChaseTimers.reserve(n);
    predatorApproachDirs.reserve(n);
    predatorDisengageDirs.reserve(n);
    predatorInfluences.reserve(n);
    predatorThreats.reserve(n);
    boidCohesionMemories.reserve(n);
    boidActiveNeighbors.reserve(n);
    boidNeighborIndices.reserve(n);
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
    predatorRestTimers.resize(n, 0.0f);
    predatorChaseTimers.resize(n, 0.0f);
    predatorApproachDirs.resize(n, glm::vec3(0.0f));
    predatorDisengageDirs.resize(n, glm::vec3(0.0f));
    predatorInfluences.resize(n, glm::vec3(0.0f));
    predatorThreats.resize(n, 0.0f);
    boidCohesionMemories.resize(n);
    boidActiveNeighbors.resize(n);
    boidNeighborIndices.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      boidNeighborIndices[i].fill(-1);
    }
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