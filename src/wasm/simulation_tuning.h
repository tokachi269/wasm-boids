#pragma once

/**
 * グローバルな逃避・群れ調整パラメータ。
 * UI から動的に変更され、全 boid に共通して適用される。
 */
struct SimulationTuningParams {
  float threatDecay = 1.6f;             // 単位: 1/sec。値が大きいほど恐怖が早く薄れる
  float maxEscapeWeight = 0.8f;         // 逃走方向に割ける最大割合（0〜1）
  float baseEscapeStrength = 5.0f;      // 逃走ステアリング（目標速度への舵取り）強度の基礎値
  float fastAttractStrength = 1.0f;     // 近傍不足時の補助凝集強度（0で無効）
  float schoolPullCoefficient = 0.0008f;// 大クラスタ引力係数

  // 散らばり過ぎ防止の「見えないソフト境界」。
  // - softBoundaryStart を超えたあたりから中心寄せが始まり、softBoundaryRadius に向けて強くなる。
  // - 反射やクランプではなく「速度の舵取り」で戻すため、境界で溜まりにくい。
  float softBoundaryRadius = 200.0f;    // 単位: m（ワールド単位）。0以下で無効。
  float softBoundaryStart = 120.0f;     // 単位: m。半径の内側でも散開抑制を少し早めに効かせたい場合に小さくする。
  float softBoundarySteer = 0.25f;      // 単位: 1/sec 目安。大きいほど中心へ戻す舵取りが強い。
};

// グローバルなチューニングパラメータ実体
extern SimulationTuningParams gSimulationTuning;
