#include "entry.h"
#include "boids_tree.h"
#include "scale_utils.h"
#include <iostream>

void Entry::run() {
  std::cout << "WebAssembly entry point initialized!" << std::endl;
}

extern "C" {

void initBoids(float pr, float vr) {
  BoidTree::instance().initializeBoids(pr, vr);
}
void build(int maxPerUnit = 16, int level = 0) {
  BoidTree::instance().build(maxPerUnit, level);
}
uintptr_t posPtr() { return BoidTree::instance().getPositionsPtr(); }
uintptr_t velPtr() { return BoidTree::instance().getVelocitiesPtr(); }
uintptr_t oriPtr() { return BoidTree::instance().getOrientationsPtr(); }
int boidCount() { return BoidTree::instance().getBoidCount(); }
void update(float dt) { BoidTree::instance().update(dt); }
void setFlockSize(int newSize, float posRange, float velRange) {
  BoidTree::instance().setFlockSize(newSize, posRange, velRange);
}
/**
 * params         … UI で入力されたそのままの値
 * spatialScale   … “今回モデルをどれだけ縮小したか”
 *                  例）モデル寸法 10 倍 → 0.1,   100 分の 1 → 0.01
 *                  迷ったら 1.0 にしておくと従来通り
 */
void setSpeciesParams(const SpeciesParams &params,
                      float spatialScale /*=1.0f*/) {
  BoidTree::instance().setGlobalSpeciesParams(scaledParams(params, spatialScale));
}
}
