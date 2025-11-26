#pragma once

/**
 * グローバルな逃避・群れ調整パラメータ。
 * UI から動的に変更され、全 boid に共通して適用される。
 */
struct SimulationTuningParams {
  float threatDecay = 1.6f;             // 単位: 1/sec。値が大きいほど恐怖が早く薄れる
  float threatGain = 1.5f;              // threat -> 逃走ブレンドへの変換倍率
  float maxEscapeWeight = 0.8f;         // 逃走方向に割ける最大割合（0〜1）
  float baseEscapeStrength = 5.0f;      // 逃走加速度の基礎値
  float escapeStrengthPerThreat = 10.0f;// threat に応じて加算する係数
  float cohesionBoost = 1.2f;           // threat に応じた凝集力強化量
  float separationMinFactor = 0.55f;    // threat=1 時の分離力スケール下限（0〜1）
  float alignmentBoost = 0.6f;          // threat に応じた整列強化量
  float fastAttractStrength = 1.0f;     // 近傍不足時の補助凝集強度（0で無効）
};

// グローバルなチューニングパラメータ実体
extern SimulationTuningParams gSimulationTuning;
