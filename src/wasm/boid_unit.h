#pragma once

#include <cstddef>

struct SoABuffers;
class LbvhIndex;

// KD 木時代との互換性確保のために残しているダミー構造体。
// LBVH 移行後はメタデータ保持に利用しない。
struct BoidUnit {};

// LBVH インデックスを用いた近傍相互作用計算。
void computeBoidInteractionsRange(SoABuffers &buf, const LbvhIndex &index,
                                  int begin, int end, float dt);

// 速度・位置・姿勢などの運動学的更新を SoA バッファ上で処理。
void updateBoidKinematicsRange(SoABuffers &buf, int begin, int end,
                               float dt);