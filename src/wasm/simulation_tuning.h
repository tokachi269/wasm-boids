#pragma once

/**
 * グローバルな逃避・群れ調整パラメータ。
 * UI から動的に変更され、全 boid に共通して適用される。
 */
struct SimulationTuningParams {
  float threatDecay = 1.0f;             // 単位: 1/sec。値が大きいほど恐怖が早く薄れる
  float maxEscapeWeight = 0.6f;         // 逃走方向に割ける最大割合（0〜1）
  float baseEscapeStrength = 6.0f;      // 逃走ステアリング（目標速度への舵取り）強度の基礎値
  float schoolPullCoefficient = 0.0002f;// 大クラスタ引力係数
  float schoolPullStartDistance = 2.5f;
  float schoolPullFullDistance = 3.0f;
  float schoolPullDenseScale = 0.16f;

  // 相互作用系の更新率。1 は毎step、0.5 は2stepに1回相当。
  // kinematics、近距離反発、separation、逃避は常に毎step更新する。
  float neighborRefreshRate = 1.0f;
  float alignmentUpdateRate = 1.0f;
  float cohesionUpdateRate = 1.0f;
  float schoolPullUpdateRate = 1.0f;
  float predatorTargetUpdateRate = 1.0f;

  // 散らばり過ぎ防止の、固定ワールド原点を基準にした「見えないソフト境界」。
  // - softBoundaryStart を超えたあたりから原点寄せが始まり、softBoundaryRadius に向けて強くなる。
  // - 反射やクランプではなく「速度の舵取り」で戻すため、境界で溜まりにくい。
  float softBoundaryRadius = 200.0f;    // 単位: m（ワールド単位）。0以下で無効。
  float softBoundaryStart = 120.0f;     // 単位: m。半径の内側でも散開抑制を少し早めに効かせたい場合に小さくする。
  float softBoundarySteer = 0.25f;      // 単位: 1/sec 目安。大きいほど中心へ戻す舵取りが強い。
};

// グローバルなチューニングパラメータ実体
extern SimulationTuningParams gSimulationTuning;
