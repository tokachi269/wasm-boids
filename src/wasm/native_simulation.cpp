#include "native_simulation.h"
#include "boids_simulation.h"
#include "platform_utils.h"
#include "boids_parallel_config.h"
#include "scale_utils.h"
#include "simulation_tuning.h"

#include <atomic>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <glm/glm.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string_view>
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

bool NativeSimulation::configureFromCommandLine(int argc, char **argv) {
  const auto parseUnsigned = [](const char *text, auto &value) {
    const std::string_view input(text);
    const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
    return result.ec == std::errc{} && result.ptr == input.data() + input.size();
  };

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg != "--bench" && arg != "--predator-diagnostic" &&
        arg != "--seed" && arg != "--boids" && arg != "--tasks") {
      std::cerr << "Unknown argument: " << arg << '\n';
      return false;
    }
    if (++i >= argc) {
      std::cerr << "Missing value for " << arg << '\n';
      return false;
    }

    if (arg == "--bench") {
      options_.bench = true;
      if (!parseUnsigned(argv[i], options_.benchFrames) || options_.benchFrames == 0) {
        std::cerr << "Invalid frame count: " << argv[i] << '\n';
        return false;
      }
    } else if (arg == "--predator-diagnostic") {
      options_.predatorDiagnostic = true;
      if (!parseUnsigned(argv[i], options_.benchFrames) || options_.benchFrames == 0) {
        std::cerr << "Invalid diagnostic frame count: " << argv[i] << '\n';
        return false;
      }
    } else if (arg == "--seed") {
      if (!parseUnsigned(argv[i], options_.seed)) {
        std::cerr << "Invalid seed: " << argv[i] << '\n';
        return false;
      }
    } else if (arg == "--boids") {
      unsigned int count = 0;
      if (!parseUnsigned(argv[i], count) || count == 0 ||
          count > static_cast<unsigned int>(std::numeric_limits<int>::max())) {
        std::cerr << "Invalid boid count: " << argv[i] << '\n';
        return false;
      }
      options_.benchBoids = static_cast<int>(count);
    } else {
      if (!parseUnsigned(argv[i], options_.benchTasks) ||
          options_.benchTasks > 64) {
        std::cerr << "Invalid task limit: " << argv[i] << '\n';
        return false;
      }
    }
  }
  return true;
}

// シミュレーション全体の起動（初期化→ループ開始）
void NativeSimulation::run() {
  std::signal(SIGINT, handleSignal); // Ctrl+C で停止可能

  if (options_.predatorDiagnostic) {
    runPredatorDiagnostic();
    return;
  }

  settings_ = ensureSettingsFields(loadSettings()); // 設定値の取得・補完
  if (options_.benchBoids > 0) {
    int nonPrimaryCount = 0;
    for (std::size_t i = 1; i < settings_.size(); ++i) {
      nonPrimaryCount += settings_[i].count;
    }
    if (options_.benchBoids < nonPrimaryCount) {
      std::cerr << "Boid count is smaller than the non-primary species count.\n";
      return;
    }
    settings_.front().count = options_.benchBoids - nonPrimaryCount;
  }
  startSimulation();                                // BoidSimulation 初期化
  if (options_.bench) {
    runBenchmark();
  } else {
    animate();                                      // メインループ開始
  }
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
  boids.cohesion = 35.0f;
  boids.separation = 0.5f;
  boids.alignment = 20.0f;
  boids.maxSpeed = 0.26f;
  boids.minSpeed = 0.0f;
  // maxTurnAngle は「最大曲率（移動距離あたりの回転量）」。
  // 角速度ではなく、speed に比例して旋回上限が決まる（速度を変えても曲がり方が崩れにくい）。
  boids.maxTurnAngle = 15.0f;
  boids.separationRange = 0.6f;
  boids.alignmentRange = 7.0f;
  boids.cohesionRange = 11.0f;
  boids.maxNeighbors = 4;
  boids.lambda = 1.0f;
  boids.tau = 1.0f;
  boids.horizontalTorque = 0.02f;
  boids.torqueStrength = 10.0f;
  boids.fieldOfViewDeg = 180.0f;
  boids.predatorAlertRadius = 1.0f;
  boids.isPredator = false;
  boids.densityReturnStrength = 50.0f;
  
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
  // 捕食者はやや鋭く旋回できる想定（曲率を高めにする）。
  predator.maxTurnAngle = 12.0f;
  predator.separationRange = 14.0f;
  predator.alignmentRange = 11.0f;
  predator.cohesionRange = 77.0f;
  predator.maxNeighbors = 0;
  predator.lambda = 0.05f;
  predator.tau = 1.0f;
  predator.horizontalTorque = 0.022f;
  predator.torqueStrength = 0.0f;
  predator.predatorAlertRadius = 0.8f;
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

// BoidSimulation の初期化（個体生成・空間分割構築）
void NativeSimulation::startSimulation() {
  std::vector<SpeciesParams> scaled;
  scaled.reserve(settings_.size());
  for (const auto &params : settings_) {
    scaled.push_back(scaledParams(params, options_.spatialScale));
  }

  world_.configure(boids::BoidsWorldConfig{
      .positionRange = options_.positionRange,
      .velocityRange = options_.velocityRange,
      .maxBoidsPerUnit = options_.maxBoidsPerUnit,
      .seed = options_.seed,
      .fixedTimeStep = options_.fixedTimeStep,
  });
  world_.reset(scaled);

  const int totalBoids = calculateTotalBoidCount(settings_);
  if (!options_.bench) {
    logger::log("Simulation initialized with " + std::to_string(totalBoids) +
                " boids.");
  }
}

void NativeSimulation::runPredatorDiagnostic() {
  constexpr float kFixedDt = 1.0f / 60.0f;
  constexpr int kPreyCount = 3;

  SpeciesParams prey;
  prey.species = "DiagnosticPrey";
  prey.count = kPreyCount;
  prey.speciesId = 0;
  prey.cohesion = 0.0f;
  prey.separation = 0.0f;
  prey.alignment = 0.0f;
  prey.maxSpeed = 0.3f;
  prey.minSpeed = 0.0f;
  prey.maxTurnAngle = 15.0f;
  prey.separationRange = 0.1f;
  prey.alignmentRange = 1.0f;
  prey.cohesionRange = 1.0f;
  prey.maxNeighbors = 2;
  prey.lambda = 1.0f;
  prey.tau = 1.0f;
  prey.fieldOfViewDeg = 360.0f;
  prey.predatorAlertRadius = 0.55f;
  prey.isPredator = false;

  SpeciesParams predator;
  predator.species = "DiagnosticPredator";
  predator.count = 1;
  predator.speciesId = 1;
  predator.maxSpeed = 0.0f;
  predator.minSpeed = 0.0f;
  predator.maxTurnAngle = 0.0f;
  predator.maxNeighbors = 0;
  predator.isPredator = true;

  BoidSimulation simulation;
  simulation.setRandomSeed(options_.seed);
  simulation.setFixedTimeStep(kFixedDt);
  simulation.setMaxBoidsPerUnit(8);
  simulation.initializeBoids({prey, predator}, 0.0f, 0.0f);

  auto &buffers = simulation.getBuffersMutable();
  buffers.positions[0] = glm::vec3(0.0f, 0.0f, 0.0f);
  buffers.positions[1] = glm::vec3(0.65f, 0.0f, 0.0f);
  buffers.positions[2] = glm::vec3(1.30f, 0.0f, 0.0f);
  buffers.positions[3] = glm::vec3(-0.40f, 0.0f, 0.0f);
  std::fill(buffers.velocities.begin(), buffers.velocities.end(),
            glm::vec3(0.0f));
  buffers.syncWriteFromRead();
  simulation.build();

  gSimulationTuning.threatDecay = 0.75f;
  gSimulationTuning.maxEscapeWeight = 0.6f;
  gSimulationTuning.baseEscapeStrength = 4.0f;
  setBoidsMaxTasksOverride(options_.benchTasks);

  std::array<int, kPreyCount> firstThreatFrame{-1, -1, -1};
  std::array<int, kPreyCount> firstDirectionFrame{-1, -1, -1};
  std::array<float, kPreyCount> maxThreat{0.0f, 0.0f, 0.0f};

  for (std::size_t frame = 0; frame < options_.benchFrames; ++frame) {
    simulation.update(kFixedDt);
    for (int i = 0; i < kPreyCount; ++i) {
      const float threat = buffers.predatorThreats[i];
      maxThreat[i] = std::max(maxThreat[i], threat);
      if (firstThreatFrame[i] < 0 && threat > 0.02f) {
        firstThreatFrame[i] = static_cast<int>(frame);
      }
      if (firstDirectionFrame[i] < 0 &&
          glm::length2(buffers.predatorInfluences[i]) > 1e-6f) {
        firstDirectionFrame[i] = static_cast<int>(frame);
      }
    }
  }

  std::ostringstream output;
  output << std::setprecision(8)
         << "{\"mode\":\"predator-diagnostic\",\"frames\":"
         << options_.benchFrames << ",\"seed\":" << options_.seed
         << ",\"tasks\":" << options_.benchTasks;
  const auto writeArray = [&output](const char *name, const auto &values) {
    output << ",\"" << name << "\":[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i != 0) output << ',';
      output << values[i];
    }
    output << ']';
  };
  writeArray("first_threat_frame", firstThreatFrame);
  writeArray("first_direction_frame", firstDirectionFrame);
  writeArray("max_threat", maxThreat);
  std::array<float, kPreyCount> finalThreat{};
  for (int i = 0; i < kPreyCount; ++i) {
    finalThreat[i] = buffers.predatorThreats[i];
  }
  writeArray("final_threat", finalThreat);
  output << '}';
  std::cout << output.str() << '\n';
}

void NativeSimulation::runBenchmark() {
  using clock = std::chrono::steady_clock;
  constexpr std::size_t kWarmupFrames = 1000;
  constexpr float kFixedDt = 1.0f / 60.0f;
  const std::size_t measuredFrames =
      options_.benchFrames > kWarmupFrames ? options_.benchFrames - kWarmupFrames : 0;
  std::vector<double> frameTimes;
  frameTimes.reserve(measuredFrames);

  options_.sleepMillis = 0;
  options_.reportInterval = 0;
  options_.fixedTimeStep = kFixedDt;
  world_.setFixedTimeStep(kFixedDt);
  setBoidsMaxTasksOverride(options_.benchTasks);
  world_.setParallelTimingEnabled(true);
  world_.resetPhaseTimings();

  for (std::size_t frame = 0; frame < options_.benchFrames; ++frame) {
    const bool sampleLocality = frame + 1 == kWarmupFrames;
    if (sampleLocality) {
      world_.beginLocalitySample();
    }
    if (frame == kWarmupFrames) {
      world_.resetPhaseTimings();
    }
    const auto start = clock::now();
    world_.step(kFixedDt);
    const auto end = clock::now();
    if (sampleLocality) {
      world_.endLocalitySample();
    }
    if (frame >= kWarmupFrames) {
      frameTimes.push_back(
          std::chrono::duration<double, std::milli>(end - start).count());
    }
  }

  if (options_.benchFrames <= kWarmupFrames) {
    world_.resetPhaseTimings();
  }

  std::sort(frameTimes.begin(), frameTimes.end());
  const auto percentile = [&frameTimes](double fraction) {
    if (frameTimes.empty()) return 0.0;
    const auto index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(frameTimes.size()))) - 1;
    return frameTimes[index];
  };
  const double mean = frameTimes.empty()
                          ? 0.0
                          : std::accumulate(frameTimes.begin(), frameTimes.end(), 0.0) /
                                static_cast<double>(frameTimes.size());
  const double maximum = frameTimes.empty() ? 0.0 : frameTimes.back();

  uint64_t checksum = 14695981039346656037ull;
  for (const glm::vec3 &position : world_.positions()) {
    for (const float component : {position.x, position.y, position.z}) {
      uint32_t bits = 0;
      std::memcpy(&bits, &component, sizeof(bits));
      for (int byte = 0; byte < 4; ++byte) {
        checksum ^= static_cast<uint8_t>(bits >> (byte * 8));
        checksum *= 1099511628211ull;
      }
    }
  }

  const auto phases = world_.phaseTimings();
  std::ostringstream output;
  output << std::setprecision(10)
         << "{\"frames\":" << options_.benchFrames
         << ",\"warmup\":" << kWarmupFrames
         << ",\"seed\":" << options_.seed
         << ",\"boids\":" << world_.boidCount()
         << ",\"frame_ms\":{\"p50\":" << percentile(0.50)
         << ",\"p95\":" << percentile(0.95)
         << ",\"p99\":" << percentile(0.99)
         << ",\"max\":" << maximum << ",\"mean\":" << mean << "}"
         << ",\"phases\":{";
  static constexpr const char *kPhaseNames[] = {
      "updateRecursive", "treeTraversal", "computeBoidInteraction",
      "predator", "kinematics", "build", "clusterUpdate", "splitMerge"};
  for (int i = 0; i < 8; ++i) {
    if (i != 0) output << ',';
    output << '\"' << kPhaseNames[i] << "\":{\"ms\":" << phases.ms[i]
           << ",\"calls\":" << phases.calls[i] << '}';
  }
  const auto parallel = world_.parallelTimings();
  output << "},\"parallel\":{\"task_limit\":" << options_.benchTasks;
  static constexpr const char *kParallelNames[] = {"computeBoidInteraction",
                                                    "kinematics"};
  for (int i = 0; i < 2; ++i) {
    const double meanTask = parallel.tasks[i] > 0
                                ? parallel.taskMs[i] /
                                      static_cast<double>(parallel.tasks[i])
                                : 0.0;
    output << ",\"" << kParallelNames[i] << "\":{\"tasks\":"
           << parallel.tasks[i] << ",\"frames\":" << parallel.frames[i]
           << ",\"mean_task_ms\":" << meanTask
           << ",\"min_task_ms\":" << parallel.minTaskMs[i]
           << ",\"max_task_ms\":" << parallel.maxTaskMs[i]
           << ",\"worst_max_over_mean\":"
           << parallel.worstMaxOverMean[i] << '}';
  }
  const auto locality = world_.localityStats();
  output << "},\"locality\":{";
  static constexpr const char *kLocalityNames[] = {"same_leaf", "external"};
  static constexpr const char *kBucketNames[] = {"le_1", "le_4", "le_16",
                                                  "le_64", "le_256", "gt_256"};
  for (int kind = 0; kind < 2; ++kind) {
    if (kind != 0) output << ',';
    const double meanDistance = locality.samples[kind] > 0
                                    ? static_cast<double>(locality.distanceSum[kind]) /
                                          static_cast<double>(locality.samples[kind])
                                    : 0.0;
    output << '\"' << kLocalityNames[kind] << "\":{\"samples\":"
           << locality.samples[kind] << ",\"mean_abs_index_delta\":"
           << meanDistance << ",\"buckets\":{";
    for (int bucket = 0; bucket < 6; ++bucket) {
      if (bucket != 0) output << ',';
      output << '\"' << kBucketNames[bucket] << "\":"
             << locality.buckets[kind][bucket];
    }
    output << "}}";
  }
  output << "},\"checksum\":\"" << std::hex << std::setw(16)
         << std::setfill('0') << checksum << "\"}";
  std::cout << output.str() << '\n';
}

// メインループ（フレームごとに BoidSimulation を更新）
void NativeSimulation::animate() {
  using clock = std::chrono::steady_clock;
  auto last = clock::now();
  std::size_t frame = 0;

  while (g_running && (options_.maxFrames == 0 || frame < options_.maxFrames)) {
    const auto now = clock::now();
    const float deltaSeconds = std::chrono::duration<float>(now - last).count();
    last = now;

    if (!paused_) {
      if (options_.fixedTimeStep > 0.0f) {
        world_.stepFixed(deltaSeconds);
      } else {
        world_.step(deltaSeconds);
      }
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
  const auto positions = world_.positions();
  const auto velocities = world_.velocities();
  if (positions.empty()) {
    logger::log("Frame " + std::to_string(frame) + ": no boids available.");
    return;
  }

  // 平均位置
  const glm::vec3 averagePosition =
      std::accumulate(positions.begin(), positions.end(),
                      glm::vec3(0.0f)) /
      static_cast<float>(positions.size());

  // 平均速度
  const glm::vec3 averageVelocity =
      std::accumulate(velocities.begin(), velocities.end(),
                      glm::vec3(0.0f)) /
      static_cast<float>(velocities.size());

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << "Frame " << frame
      << " | dt=" << deltaSeconds << " | avgPos=(" << averagePosition.x << ", "
      << averagePosition.y << ", " << averagePosition.z << ")"
      << " | avgVel=(" << averageVelocity.x << ", " << averageVelocity.y << ", "
      << averageVelocity.z << ")";

  logger::log(oss.str());
}
