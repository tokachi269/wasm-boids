#include "native_simulation.h"
#include "boids_tree.h"
#include "platform_utils.h"
#include "scale_utils.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <glm/glm.hpp>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <thread>

namespace {
// シミュレーションループの継続フラグ（Ctrl+C で false になる）
std::atomic<bool> g_running{true};

// SIGINT (Ctrl+C) 受信時にループを停止
void handleSignal(int) { g_running = false; }

bool isLowPowerDevice() {
  const unsigned int concurrency = std::thread::hardware_concurrency();
  return concurrency > 0 && concurrency <= 4;
}
} // namespace

// コンストラクタ: 低スペック環境なら sleepMillis を増やす
NativeSimulation::NativeSimulation() {
  if (isLowPowerDevice()) {
    // 低スペック環境ではループ間隔を少し伸ばす
    options_.sleepMillis = 20;
  }
}

// シミュレーション全体の起動（初期化→ループ開始）
void NativeSimulation::run() {
  std::signal(SIGINT, handleSignal); // Ctrl+C で停止可能

  settings_ = ensureSettingsFields(loadSettings()); // 設定値の取得・補完
  startSimulation();                                // BoidTree 初期化
  animate();                                        // メインループ開始
}

// デフォルトの種パラメータ（Boids/Predator）を返す
std::vector<SpeciesParams> NativeSimulation::getDefaultSettings() const {
  const bool lowPower = isLowPowerDevice();
  const int boidCount = lowPower ? 3000 : 10000;

  std::vector<SpeciesParams> defaults;
  // 通常 Boids
  SpeciesParams boids;
  boids.species = "Boids";
  boids.count = boidCount;
  boids.speciesId = 0;
  boids.cohesion = 30.0f;
  boids.separation = 8.0f;
  boids.alignment = 17.0f;
  boids.maxSpeed = 0.26f;
  boids.minSpeed = 0.0f;
  boids.maxTurnAngle = 0.25f;
  boids.separationRange = 0.6f;
  boids.alignmentRange = 6.0f;
  boids.cohesionRange = 30.0f;
  boids.maxNeighbors = 6;
  boids.lambda = 0.62f;
  boids.tau = 1.5f;
  boids.horizontalTorque = 0.019f;
  boids.torqueStrength = 10.0f;
  boids.fieldOfViewDeg = 180.0f;
  boids.isPredator = false;
  
  defaults.push_back(boids);

  // 捕食者
  SpeciesParams predator;
  predator.species = "Predator";
  predator.count = 1;
  predator.speciesId = 1;
  predator.cohesion = 5.58f;
  predator.separation = 0.0f;
  predator.alignment = 0.0f;
  predator.maxSpeed = 1.37f;
  predator.minSpeed = 0.4f;
  predator.maxTurnAngle = 0.2f;
  predator.separationRange = 14.0f;
  predator.alignmentRange = 11.0f;
  predator.cohesionRange = 77.0f;
  predator.maxNeighbors = 0;
  predator.lambda = 0.05f;
  predator.tau = 1.0f;
  predator.horizontalTorque = 0.022f;
  predator.torqueStrength = 0.0f;
  predator.isPredator = true;

  defaults.push_back(predator);
  return defaults;
}

// 設定値の不足項目をデフォルトで補完
std::vector<SpeciesParams> NativeSimulation::ensureSettingsFields(
    std::vector<SpeciesParams> settingsArray) const {
  const auto defaults = getDefaultSettings();

  for (auto &setting : settingsArray) {
    // 種名一致でデフォルト検索
    const auto *fallback = [&]() -> const SpeciesParams * {
      for (const auto &def : defaults) {
        if (def.species == setting.species) {
          return &def;
        }
      }
      return defaults.empty() ? nullptr : &defaults.front();
    }();

    if (!fallback) {
      continue;
    }

    // lambda, tau が未設定ならデフォルト値で補完
    if (setting.lambda == 0.0f) {
      setting.lambda = fallback->lambda;
    }
    if (setting.tau == 0.0f) {
      setting.tau = fallback->tau;
    }
  }

  return settingsArray;
}

// 設定値のロード（JS版はlocalStorage、ネイティブはデフォルトのみ）
std::vector<SpeciesParams> NativeSimulation::loadSettings() const {
  // JavaScript 版では localStorage から取得していたが、
  // ネイティブ版ではデフォルト設定を使用する。
  return getDefaultSettings();
}

// 全 Boid 数を合計
int NativeSimulation::calculateTotalBoidCount(
    const std::vector<SpeciesParams> &settingsArray) const {
  int sum = 0;
  for (const auto &setting : settingsArray) {
    sum += setting.count;
  }
  return sum;
}

// BoidTree の初期化（個体生成・空間分割構築）
void NativeSimulation::startSimulation() {
  std::vector<SpeciesParams> scaled;
  scaled.reserve(settings_.size());
  for (const auto &params : settings_) {
    scaled.push_back(scaledParams(params, options_.spatialScale));
  }

  BoidTree::instance().initializeBoids(scaled, options_.positionRange,
                                       options_.velocityRange);
  BoidTree::instance().build(options_.maxBoidsPerUnit, options_.rootLevel);

  const int totalBoids = calculateTotalBoidCount(settings_);
  logger::log("Simulation initialized with " + std::to_string(totalBoids) +
              " boids.");
}

// メインループ（フレームごとに BoidTree を更新）
void NativeSimulation::animate() {
  using clock = std::chrono::steady_clock;
  auto last = clock::now();
  std::size_t frame = 0;

  while (g_running && (options_.maxFrames == 0 || frame < options_.maxFrames)) {
    const auto now = clock::now();
    const float deltaSeconds = std::chrono::duration<float>(now - last).count();
    last = now;

    if (!paused_) {
      BoidTree::instance().update(deltaSeconds); // BoidTree の物理更新
    }

    // 指定間隔ごとに統計ログ出力
    if (options_.reportInterval > 0 && frame % options_.reportInterval == 0) {
      printFrameSummary(frame, deltaSeconds);
    }

    ++frame;
    scheduleNextFrame(); // スリープ
  }

  logger::log("Simulation loop stopped.");
}

// フレーム間スリープ（FPS調整）
void NativeSimulation::scheduleNextFrame() {
  if (options_.sleepMillis > 0) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(options_.sleepMillis));
  }
}

// フレームごとの統計（平均位置・速度）を出力
void NativeSimulation::printFrameSummary(std::size_t frame,
                                         float deltaSeconds) const {
  const auto &buf = BoidTree::instance().buf;
  if (buf.positions.empty()) {
    logger::log("Frame " + std::to_string(frame) + ": no boids available.");
    return;
  }

  // 平均位置
  const glm::vec3 averagePosition =
      std::accumulate(buf.positions.begin(), buf.positions.end(),
                      glm::vec3(0.0f)) /
      static_cast<float>(buf.positions.size());

  // 平均速度
  const glm::vec3 averageVelocity =
      std::accumulate(buf.velocities.begin(), buf.velocities.end(),
                      glm::vec3(0.0f)) /
      static_cast<float>(buf.velocities.size());

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << "Frame " << frame
      << " | dt=" << deltaSeconds << " | avgPos=(" << averagePosition.x << ", "
      << averagePosition.y << ", " << averagePosition.z << ")"
      << " | avgVel=(" << averageVelocity.x << ", " << averageVelocity.y << ", "
      << averageVelocity.z << ")";

  logger::log(oss.str());
}
