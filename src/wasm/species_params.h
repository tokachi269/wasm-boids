#pragma once
#include <string>

struct SpeciesParams {
  std::string species;
  int count = 100; // 個体数
  int speciesId = 1; // 種の識別子
  float cohesion = 0.01f;
  float separation = 0.1f;
  float alignment = 0.05f;
  float maxSpeed = 1.0f;
  float minSpeed = 0.1f;
  float maxTurnAngle = 0.01f;
  float separationRange = 10.0f;
  float alignmentRange = 30.0f;
  float cohesionRange = 50.0f;
  int maxNeighbors = 4;
  float lambda = 0.05f;
  float tau = 0.2f;
  float horizontalTorque = 0.005f;
  float velocityEpsilon = 0.0001f;
  float torqueStrength = 0.1f;
  float fieldOfViewDeg = 180.0f;
  float bodyHeadLength = -0.15f; // 先端までの距離（正負は魚のローカル前方基準）
  float bodyTailLength = 0.33f; // 後端までの距離
  float bodyRadius = 0.035f;    // 直径0.07mの半径
  bool isPredator = false;
};