#pragma once

#include "species_params.h"
#include <vector>

class NativeSimulation {
public:
  NativeSimulation();
  void run();

private:
  struct Options {
    float spatialScale = 1.0f;
    float positionRange = 3.0f;
    float velocityRange = 0.25f;
    int maxBoidsPerUnit = 16;
    int rootLevel = 0;
    int reportInterval = 60; // フレーム数
    int sleepMillis = 16;
    std::size_t maxFrames = 600; // 約10秒 (60FPS想定)
  };

  std::vector<SpeciesParams> settings_;
  Options options_;
  bool paused_ = false;

  std::vector<SpeciesParams> getDefaultSettings() const;
  std::vector<SpeciesParams>
  ensureSettingsFields(std::vector<SpeciesParams> settingsArray) const;
  std::vector<SpeciesParams> loadSettings() const;
  int calculateTotalBoidCount(
      const std::vector<SpeciesParams> &settingsArray) const;

  void startSimulation();
  void animate();
  void scheduleNextFrame();
  void printFrameSummary(std::size_t frame, float deltaSeconds) const;
};
