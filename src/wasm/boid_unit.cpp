#include "boid_unit.h"

#include "boids_buffers.h"
#include "boids_tree.h"
#include "obstacle_field.h"
#include "pool_accessor.h"
#include "simulation_tuning.h"
#include "spatial_query.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <future>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <limits>
#include <stack>
#include <thread>
#include <utility>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten/threading.h>
#endif


namespace {

constexpr float kLeafDensityMinRadius = 0.25f;
// simpleDensity の計算モード（0: 重心法、1: 全ペア、2: サンプルペア）
constexpr int kLeafSimpleDensityMode = 1;
constexpr float kLeafSimpleDensityEpsilon = 1e-4f;
constexpr std::size_t kCandidateCacheLimit = 16384; // thread-local reuse上限
constexpr std::size_t kPredatorCacheLimit = 2048;
constexpr float kPredatorChaseScale = 6.0f;   // τ から追跡可能時間を算出する係数
constexpr float kPredatorRestScale = 3.0f;    // τ から休憩時間を算出する係数
constexpr float kPredatorChaseMin = 3.5f;     // 追跡時間の下限（秒）
constexpr float kPredatorChaseMax = 10.0f;    // 追跡時間の上限（秒）
constexpr float kPredatorRestMin = 2.0f;      // 休憩時間の下限（秒）
constexpr float kPredatorRestMax = 7.0f;      // 休憩時間の上限（秒）
constexpr float kPredatorRestSpeedScale = 0.45f; // 休憩時に維持する速度スケール
constexpr float kTwoPi = 6.28318530718f;

// ------------------------------------------------------------
// NaN/Inf を「作らない」ための最小限ガード
// ------------------------------------------------------------
// - 0ベクトルの正規化を避け、以降の計算へ NaN が伝播するのを防ぐ。
// - isfinite のような重い判定は避け、length2 の閾値チェックだけで済ませる。
constexpr float kSafeNormalizeEps2 = 1e-12f;

inline glm::vec3 safeNormalizeOr(const glm::vec3 &v,
                                 const glm::vec3 &fallback) {
  const float len2 = glm::length2(v);
  if (!(len2 > kSafeNormalizeEps2)) {
    return fallback;
  }
  return v * (1.0f / glm::sqrt(len2));
}

inline float fastHash01(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return float(x & 0x00ffffffu) * (1.0f / 16777215.0f);
}

inline bool tryNormalizeXZ(const glm::vec3 &v, glm::vec3 &out) {
  const float xzLen2 = v.x * v.x + v.z * v.z;
  if (!(xzLen2 > kSafeNormalizeEps2)) {
    return false;
  }
  out = glm::vec3(v.x, 0.0f, v.z) * (1.0f / glm::sqrt(xzLen2));
  return true;
}

float computeLeafSimpleDensity(const int *indices, std::size_t count,
                               const SoABuffers &buffers,
                               const SpeciesParams &speciesParams) {
  if (!indices || count == 0) {
    return 0.0f;
  }

  const float baseRange =
      glm::max(speciesParams.separationRange * 0.5f, kLeafDensityMinRadius);
  const float smoothing = baseRange > 1e-5f ? baseRange : kLeafDensityMinRadius;
  const float smoothingSq = smoothing * smoothing + kLeafSimpleDensityEpsilon;

  if (count <= 1) {
    return 1.0f / smoothingSq;
  }

  if (kLeafSimpleDensityMode == 0) {
    glm::vec3 centroid(0.0f);
    for (std::size_t i = 0; i < count; ++i) {
      centroid += buffers.positions[indices[i]];
    }
    const float invCount = 1.0f / static_cast<float>(count);
    centroid *= invCount;

    float sumSq = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
      glm::vec3 delta = buffers.positions[indices[i]] - centroid;
      sumSq += glm::dot(delta, delta);
    }
    const float meanSq = sumSq * invCount;
    return 1.0f / (meanSq + smoothingSq);
  }

  if (kLeafSimpleDensityMode == 1) {
    const float invPairs =
        2.0f /
        (static_cast<float>(count) * static_cast<float>(count - 1));
    float accum = 0.0f;
    for (std::size_t i = 0; i + 1 < count; ++i) {
      const glm::vec3 &a = buffers.positions[indices[i]];
      for (std::size_t j = i + 1; j < count; ++j) {
        glm::vec3 diff = a - buffers.positions[indices[j]];
        float distSq = glm::dot(diff, diff);
        accum += 1.0f / (distSq + smoothingSq);
      }
    }
    return accum * invPairs;
  }

  static constexpr std::pair<int, int> kSamplePairs[] = {
      {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3},
      {4, 5}, {4, 6}, {5, 6}, {6, 7}, {2, 5}, {3, 7}};
  float accum = 0.0f;
  float weight = 0.0f;
  for (const auto &pair : kSamplePairs) {
    if (pair.first >= static_cast<int>(count) ||
        pair.second >= static_cast<int>(count)) {
      continue;
    }
    const glm::vec3 &a = buffers.positions[indices[pair.first]];
    const glm::vec3 &b = buffers.positions[indices[pair.second]];
    glm::vec3 diff = a - b;
    float distSq = glm::dot(diff, diff);
    accum += 1.0f / (distSq + smoothingSq);
    weight += 1.0f;
  }
  if (weight <= 0.0f) {
    return 1.0f / smoothingSq;
  }
  return accum / weight;
}

} // namespace

bool BoidUnit::isBoidUnit() const { return children.empty(); }

/**
 * 小角度近似を使用した軽量回転関数。
 *
 * 近似条件: sinθ ≈ θ, cosθ ≈ 1
 * 軸がほぼゼロベクトルの場合は回転をスキップ
 */
inline glm::vec3 approxRotate(const glm::vec3 &v, const glm::vec3 &axis,
                              float angle) {
  // 軽量化: 小角度近似をさらに簡略化
  // sinθ ≈ θ, cosθ ≈ 1 を前提に、外積計算を最小化
  constexpr float EPSILON = 1e-6f;
  float axisLength2 = glm::length2(axis);

  // 軸がほぼゼロベクトルの場合は回転をスキップ
  if (axisLength2 < EPSILON) {
    return v;
  }

  // 軸を正規化して回転を計算
  glm::vec3 normalizedAxis = axis * (1.0f / glm::sqrt(axisLength2));
  return v + angle * glm::cross(normalizedAxis, v);
}

// 魚体の衝突判定をカプセルで近似する際の端点情報。
struct CapsuleSegment {
  glm::vec3 a;  // 尾側端点
  glm::vec3 b;  // 頭側端点
  float radius; // 体の半径
};

// 姿勢クォータニオンから前方ベクトルを取り出す。数値が崩れた場合は Z+
// にフォールバック。
inline glm::vec3 forwardFromOrientation(const glm::quat &q) {
  glm::vec3 forward = glm::rotate(q, glm::vec3(0.0f, 0.0f, 1.0f));
  float len2 = glm::length2(forward);
  if (len2 < 1e-10f) {
    return glm::vec3(0.0f, 0.0f, 1.0f);
  }
  return forward * (1.0f / glm::sqrt(len2));
}

// 位置・姿勢・魚種パラメータからカプセルを生成する。
inline CapsuleSegment makeCapsule(const glm::vec3 &pos, const glm::quat &ori,
                                  const SpeciesParams &params) {
  glm::vec3 forward = forwardFromOrientation(ori);
  float headOffset = params.bodyHeadLength;
  float tailOffset = params.bodyTailLength;
  // headOffset
  // は前方正負どちらの指定も許容し、魚種ごとの原点位置ズレを吸収する。
  glm::vec3 head = pos + forward * headOffset;
  glm::vec3 tail = pos - forward * tailOffset;
  float radius = std::max(params.bodyRadius, 0.0f);
  return {tail, head, radius};
}

// 2線分間の最接近点を求め、平方距離を返すワーク関数。
inline float closestPointsOnSegments(const glm::vec3 &p1, const glm::vec3 &q1,
                                     const glm::vec3 &p2, const glm::vec3 &q2,
                                     glm::vec3 &c1, glm::vec3 &c2) {
  constexpr float EPS = 1e-6f;
  glm::vec3 d1 = q1 - p1;
  glm::vec3 d2 = q2 - p2;
  glm::vec3 r = p1 - p2;
  float a = glm::dot(d1, d1);
  float e = glm::dot(d2, d2);
  float f = glm::dot(d2, r);
  float s, t;

  if (a <= EPS && e <= EPS) {
    c1 = p1;
    c2 = p2;
    return glm::length2(c1 - c2);
  }

  if (a <= EPS) {
    s = 0.0f;
    t = glm::clamp(f / e, 0.0f, 1.0f);
  } else {
    float c = glm::dot(d1, r);
    if (e <= EPS) {
      t = 0.0f;
      s = glm::clamp(-c / a, 0.0f, 1.0f);
    } else {
      float b = glm::dot(d1, d2);
      float denom = a * e - b * b;
      if (denom > EPS) {
        s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
      } else {
        s = 0.0f;
      }
      t = (b * s + f) / e;
      if (t < 0.0f) {
        t = 0.0f;
        s = glm::clamp(-c / a, 0.0f, 1.0f);
      } else if (t > 1.0f) {
        t = 1.0f;
        s = glm::clamp((b - c) / a, 0.0f, 1.0f);
      }
    }
  }

  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
  return glm::length2(c1 - c2);
}

// cross(oldDir, newDir) が退化（ほぼ平行/反平行）した場合でも、
// 旋回制限が必ず働くようにフォールバック軸を生成する。
// - oldDir は単位ベクトル想定。
// - 戻り値も単位ベクトル。
static inline glm::vec3 buildFallbackTurnAxis(const glm::vec3 &oldDir) {
  // oldDir と直交しやすい基準軸を選ぶ
  const glm::vec3 up(0.0f, 1.0f, 0.0f);
  const glm::vec3 right(1.0f, 0.0f, 0.0f);

  glm::vec3 axis = glm::cross(oldDir, up);
  float axisLen2 = glm::length2(axis);
  if (axisLen2 < 1e-8f) {
    axis = glm::cross(oldDir, right);
    axisLen2 = glm::length2(axis);
  }
  if (axisLen2 < 1e-12f) {
    // oldDir が NaN などで壊れている異常ケース。とにかく固定軸で返す。
    return glm::vec3(0.0f, 1.0f, 0.0f);
  }
  return axis * (1.0f / glm::sqrt(axisLen2));
}

static void updateLeafKinematics(BoidUnit *unit, float dt) {
  const float framePhaseBase =
      static_cast<float>(BoidTree::instance().frameCount);
  for (size_t i = 0; i < unit->indices.size(); ++i) {
    int gIdx = unit->indices[i];
    int sid = unit->buf->speciesIds[gIdx];
    glm::vec3 velocity = unit->buf->velocities[gIdx];
    glm::vec3 acceleration = unit->buf->accelerations[gIdx];
    glm::vec3 position = unit->buf->positions[gIdx];
    const bool isPredator = globalSpeciesParams[sid].isPredator;
    const float predatorRestTimer =
      isPredator ? unit->buf->predatorRestTimers[gIdx] : 0.0f;
    const float predatorChaseTimer =
      isPredator ? unit->buf->predatorChaseTimers[gIdx] : 0.0f;
    const bool predatorOnBreak = isPredator && predatorRestTimer > 0.0f;
    const float predatorWarmup =
      isPredator ? glm::clamp(predatorChaseTimer * 0.3f, 0.2f, 1.0f) : 1.0f;

    // 障害物（まずは地面）とのソフト制約を評価し、事前に逃がす。
    const glm::vec3 obstacleAvoidance =
        obstacle_field::computeAvoidance(position, velocity);
    if (obstacleAvoidance.x != 0.0f || obstacleAvoidance.y != 0.0f ||
        obstacleAvoidance.z != 0.0f) {
      acceleration += obstacleAvoidance;
    }

    // UI から渡される「逃避優先度」。
    // 低値域はデッドゾーンで0扱いにする。
    // - 逃避優先度は“逃避処理のON/OFF兼スケール”として扱い、別ルートで逃げないようにする。
    // - isPredator の個体には適用しない（捕食者側の追跡/休憩は別の目的）。
    const float rawEscapeWeight =
      glm::clamp(gSimulationTuning.maxEscapeWeight, 0.0f, 1.0f);
    constexpr float kEscapeWeightDeadzone = 0.02f;
    const float effectiveEscapeWeight =
      (rawEscapeWeight <= kEscapeWeightDeadzone)
        ? 0.0f
        : (rawEscapeWeight - kEscapeWeightDeadzone) / (1.0f - kEscapeWeightDeadzone);
    const bool escapeFeatureEnabled = !isPredator && (effectiveEscapeWeight > 0.0f);

    // -----------------------------------------------
    // 捕食者専用の追跡加速度を加算
    // -----------------------------------------------
    if (globalSpeciesParams[sid].isPredator && !predatorOnBreak &&
      unit->buf->predatorTargetIndices[gIdx] >= 0) {
      const int tgtIdx = unit->buf->predatorTargetIndices[gIdx];
      // ターゲットが不正な場合は追跡加速を適用しない。
      // （この後のターゲット再選択ロジックで置き換わる想定）
      if (tgtIdx < 0 || tgtIdx >= static_cast<int>(unit->buf->positions.size())) {
        // no-op
      } else if (unit->buf->speciesIds[tgtIdx] < 0 ||
                 unit->buf->speciesIds[tgtIdx] >=
                     static_cast<int>(globalSpeciesParams.size()) ||
                 globalSpeciesParams[unit->buf->speciesIds[tgtIdx]].isPredator) {
        // no-op
      } else {
        const glm::vec3 tgtPos = unit->buf->positions[tgtIdx];
      glm::vec3 diff = tgtPos - position;
      float d2 = glm::dot(diff, diff);
      if (d2 > 1e-4f) {
        float dist = glm::sqrt(d2);
        glm::vec3 chaseDir = diff / dist;
        float desiredSpeed = globalSpeciesParams[sid].maxSpeed;

        glm::vec3 desiredVel = chaseDir * desiredSpeed;
        glm::vec3 chaseAcc = (desiredVel - velocity) * predatorWarmup;
        acceleration += chaseAcc;
      }
      }
    } else if (predatorOnBreak) {
      // 休憩（離脱）中: 直近の獲物中心から「離れる」方向へ進ませる。
      // 方向が未設定の場合のみ、従来の wander にフォールバック。
      glm::vec3 disengage = unit->buf->predatorDisengageDirs[gIdx];
      const float d2 = glm::length2(disengage);
      glm::vec3 restDir;
      if (d2 > 1e-6f) {
        restDir = disengage * (1.0f / glm::sqrt(d2));
      } else {
        const float wanderSeed = fastHash01(uint32_t(gIdx) * 747796405u);
        const float wanderPhase = wanderSeed * kTwoPi + framePhaseBase * 0.05f;
        restDir = glm::vec3(std::cos(wanderPhase), 0.0f, std::sin(wanderPhase));
      }
      const float restSpeed =
          globalSpeciesParams[sid].maxSpeed * kPredatorRestSpeedScale * 0.6f;
      const glm::vec3 desiredVel = restDir * restSpeed;
      acceleration = glm::mix(acceleration, desiredVel - velocity, 0.45f);
    } else if (globalSpeciesParams[sid].isPredator &&
               unit->buf->predatorTargetIndices[gIdx] < 0) {
      // ターゲット未確定: 群れ（獲物密集）へ突っ込むフェーズ
      const glm::vec3 approach = unit->buf->predatorApproachDirs[gIdx];
      const float a2 = glm::length2(approach);
      if (a2 > 1e-6f) {
        const glm::vec3 approachDir = approach * (1.0f / glm::sqrt(a2));
        const float desiredSpeed = globalSpeciesParams[sid].maxSpeed * 0.85f;
        const glm::vec3 desiredVel = approachDir * desiredSpeed;
        acceleration += (desiredVel - velocity) * predatorWarmup * 0.75f;
      }
     } else if (unit->buf->stresses[gIdx] > 0.1f ||
       unit->buf->predatorThreats[gIdx] > 0.02f ||
       glm::length2(unit->buf->predatorInfluences[gIdx]) > (0.001f * 0.001f)) {
      // -----------------------------------------------
      // 逃避挙動: 捕食者からの影響を逃走加速度に変換
      // -----------------------------------------------
      // 逃避優先度が 0（デッドゾーン含む）の場合、捕食者由来の処理自体が不要。
      // ここでは「恐怖(threat)の減衰」だけ行い、逃避方向計算などはスキップする。
      // 目的:
      // - ユーザー期待: 逃避OFFなら捕食者由来の効果が走らない
      // - 性能: 影響ベクトルの正規化や目標速度計算をホットループから外す
      if (!escapeFeatureEnabled) {
      unit->buf->predatorThreats[gIdx] = glm::max(
        unit->buf->predatorThreats[gIdx] - gSimulationTuning.threatDecay * dt,
        0.0f);
      } else {
      float currentStress = unit->buf->stresses[gIdx];
      glm::vec3 storedInfluence = unit->buf->predatorInfluences[gIdx];
      // `length()` は内部で sqrt を行うため、まず length2 を取り、必要なときだけ sqrt する。
      const float predatorInfluenceMagnitude2 = glm::length2(storedInfluence);
      const float predatorInfluenceMagnitude =
        (predatorInfluenceMagnitude2 > 0.0f)
          ? glm::sqrt(predatorInfluenceMagnitude2)
          : 0.0f;

      // threatState は applyPredatorSweep で蓄積した恐怖レベル (0-1)。
      float threatState = unit->buf->predatorThreats[gIdx];
      // 直接の影響が強い場合は threatState を補強（0-1 に正規化して混ぜる）。
      float influenceThreat =
        glm::clamp(predatorInfluenceMagnitude * 0.15f, 0.0f, 1.0f);
      threatState = glm::max(threatState, influenceThreat);
      float threatLevel = glm::clamp(threatState, 0.0f, 1.0f);

      // 逃避ブレンドは「閾値で突然ON」にすると体感として“反応が遅い”になりやすい。
      // そこで threatLevel に応じて滑らかに立ち上げる。
      // - 低脅威: 小さく回避し始める（群れ行動を壊さない）
      // - 高脅威: 逃避を優先（分離/整列/凝集との干渉を抑える）
      // predatorInfluences は後段で減衰するため、ヒステリシスとしても効く。
      constexpr float kEscapeStartThreat = 0.08f;
      constexpr float kEscapeFullThreat = 0.25f;
      float escapeBlend = 0.0f;
      if (threatLevel >= kEscapeStartThreat) {
        float t = (threatLevel - kEscapeStartThreat) / (kEscapeFullThreat - kEscapeStartThreat);
        t = glm::clamp(t, 0.0f, 1.0f);
        // smoothstep: t^2*(3-2t)
        const float smoothT = t * t * (3.0f - 2.0f * t);
        escapeBlend = smoothT * effectiveEscapeWeight;
      }
      // 直接の影響が十分強い場合は即座にフル逃避へ。
      if (predatorInfluenceMagnitude > 0.35f) {
        escapeBlend = effectiveEscapeWeight;
      }

      // 逃避加速度を計算（捕食者から遠ざかる方向）
        glm::vec3 escapeForce = storedInfluence;
        if (predatorInfluenceMagnitude > 0.001f) {
          glm::vec3 fleeDir = storedInfluence / predatorInfluenceMagnitude;
        // 逃避を「一定加速度を付ける」ではなく「目標速度へ舵取りする」形にする。
        // - 速度がすでに逃走方向に乗っている場合は余計な加速が減り、挙動が滑らかになる。
        // - threat が高いほど、目標速度を少し上げて危険時のキビキビ感を出す。
        // 逃避を「一定加速度」ではなく「目標速度へ舵取り」で表現する。
        // threatLevel が高いほど、舵取りも目標速度も少し強める。
        const float fleeStrength =
          gSimulationTuning.baseEscapeStrength * (0.65f + 1.10f * threatLevel) *
          (1.0f + 0.2f * currentStress);
        const float desiredSpeed =
            globalSpeciesParams[sid].maxSpeed * (1.0f + 0.65f * threatLevel);
        const glm::vec3 desiredVel = fleeDir * desiredSpeed;
          escapeForce = (desiredVel - velocity) * fleeStrength;
        }

      // 通常加速度と逃避加速度の合成
      // - 低脅威: 小さくブレンドして“早めに”回避を開始
      // - 高脅威: 逃避ブレンドが 1 に近づき、逃避を優先
      if (escapeBlend > 0.0f) {
        acceleration =
            acceleration * (1.0f - escapeBlend) + escapeForce * escapeBlend;
      }

      // threat レベルを時間経過で減衰
        float decayedThreat =
            glm::max(threatState - gSimulationTuning.threatDecay * dt, 0.0f);
        unit->buf->predatorThreats[gIdx] = decayedThreat;
      }
    } else {
      // ストレスも影響もない場合は threat を徐々に減衰
      unit->buf->predatorThreats[gIdx] = glm::max(
          unit->buf->predatorThreats[gIdx] - gSimulationTuning.threatDecay * dt,
          0.0f);
    }
    float currentStress = unit->buf->stresses[gIdx];
    float stressFactor = 1.0f;

    // 逃避優先度が0（デッドゾーン含む）の場合、
    // 捕食者由来の恐怖/ストレスを速度・旋回へ反映しない（逃避OFF扱い）。
    if (!globalSpeciesParams[sid].isPredator && !escapeFeatureEnabled) {
      currentStress = 0.0f;
      stressFactor = 1.0f;
    }

    if (currentStress > 0.95f) {
      float t = (currentStress - 0.8f) / 0.2f;
      stressFactor = 1.0f + 1.6f + t * 0.9f;
    } else if (currentStress > 0.7f) {
      float t = (currentStress - 0.2f) / 0.6f;
      float smoothT = t * t * (3.0f - 2.0f * t);
      stressFactor = 1.0f + 0.4f + smoothT * 1.8f;
    } else {
      float t = currentStress / 0.2f;
      stressFactor = 1.0f + t * 0.4f;
    }

    float maxSpeed = globalSpeciesParams[sid].maxSpeed * stressFactor;
    if (predatorOnBreak) {
      maxSpeed *= kPredatorRestSpeedScale;
    }

    // -----------------------------------------------
    // 全体ガガガ対策:
    // 捕食者逃避 + 密集反発などが重なると、加速度が極端になって方向計算が暴れることがある。
    // 「1ステップで速度変化が maxSpeed*2 を超えない」範囲に加速度をクリップする。
    // length2 比較で弾き、必要なときだけ sqrt/inversesqrt を使う。
    // -----------------------------------------------
    if (dt > 0.0f && maxSpeed > 0.0f) {
      const float a2 = glm::length2(acceleration);
      // |dv| = |a|*dt <= maxSpeed*2
      const float dvMax = maxSpeed * 2.0f;
      const float a2Max = (dvMax * dvMax) / (dt * dt);
      if (a2 > a2Max) {
        const float invA = 1.0f / glm::sqrt(a2);
        const float aMax = dvMax / dt;
        acceleration *= (aMax * invA);
      }
    }

    // -----------------------------------------------
    // 共通処理: 速度予測と回転角制限
    // -----------------------------------------------
    glm::vec3 desiredVelocity = velocity + acceleration * dt;
    // 極小ベクトルの normalize はノイズを増幅して急反転/震えになりやすいので、
    // 実用的な閾値に引き上げる。
    constexpr float MIN_DIR_LEN2 = 1e-8f;

    // 平方根を避けて速度ゼロ判定しつつ方向を確定
    float oldSpeedSq = glm::length2(velocity);
    glm::vec3 oldDir = velocity;
    if (oldSpeedSq > MIN_DIR_LEN2) {
      oldDir /= glm::sqrt(oldSpeedSq);
    } else {
      oldDir = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    // `normalize` + `length` の重複 sqrt を避けてコスト削減
    float desiredSpeedSq = glm::length2(desiredVelocity);
    float speed = 0.0f;
    glm::vec3 newDir = oldDir;
    if (desiredSpeedSq > MIN_DIR_LEN2) {
      speed = glm::sqrt(desiredSpeedSq);
      newDir = desiredVelocity / speed;
    }

    // 旋回制限は「角度(acos)」を直接求めず、cos しきい値で判定する。
    // - `acosf` は高コストなので、ホットループでは避ける。
    // - 判定自体は angle > maxTurnStep  <=>  dot < cos(maxTurnStep) で同値。
    const float dotProduct = glm::dot(oldDir, newDir);
    const float clampedDot = glm::clamp(dotProduct, -1.0f, 1.0f);
    float maxTurnCurvature = globalSpeciesParams[sid].maxTurnAngle * stressFactor;

    if (globalSpeciesParams[sid].isPredator && !predatorOnBreak &&
        unit->buf->predatorTargetIndices[gIdx] >= 0) {
      maxTurnCurvature *= 1.5f; // 捕食者の追跡時は回転制限を緩和
    }
    if (currentStress > 0.7f) {
      // 緊急時の旋回能力ブースト。
      // 過剰なブーストは方向の過修正を招きやすいので、上限を抑える。
      // currentStress ∈ [0.7, 1.0] -> factor ∈ [1.0, 1.6]
      float emergencyTurnFactor =
          1.0f + (currentStress - 0.7f) * (0.6f / 0.3f);
      maxTurnCurvature *= emergencyTurnFactor;
    }

    // 1ステップで許容する旋回角(rad)。
    // dt が大きいフレームで maxTurnAngle が高い場合、1ステップで180°近い反転が可能になり、
    // 分離/逃避などのベクトルが揺れると「左右に振り子」のようなブルブルが起きやすい。
    // そのため上限を π ではなく控えめな角度に抑える。
    constexpr float kMaxTurnStepLimit = 1.2f; // 約69°
    // speed がゼロ付近のときは方向ノイズだけが回ると破綻しやすいので、極小値で下駄を履かせる。
    const float speedForTurn = glm::max(speed, 1e-4f);
    const float maxTurnStep = glm::clamp(maxTurnCurvature * speedForTurn * dt, 0.0f, kMaxTurnStepLimit);

    // maxTurnStep が 0 のときは無駄な軸計算を避ける。
    if (maxTurnStep > 0.0f && clampedDot < std::cos(maxTurnStep)) {
      glm::vec3 axis = glm::cross(oldDir, newDir);
      float axisLength2 = glm::length2(axis);
      if (axisLength2 > 1e-8f) {
        axis /= glm::sqrt(axisLength2);
      } else {
        // 反平行などで cross が退化すると、旋回制限が効かずに暴れる。
        axis = buildFallbackTurnAxis(oldDir);
      }
      // ここに到達している時点で angle > maxTurnStep が確定しているため、回転量は maxTurnStep 固定でよい。
      newDir = approxRotate(oldDir, axis, maxTurnStep);
    }

    // -----------------------------------------------
    // 共通処理: 水平トルク（tilt補正）
    // -----------------------------------------------
    float tilt = newDir.y;
    if (fabsf(tilt) > 1e-4f) {
      // newDir がほぼ真上/真下を向くと (x,z) がゼロになり normalize が NaN になり得る。
      // その場合は水平化トルクをスキップして姿勢を壊さない。
      glm::vec3 flatDir;
      if (tryNormalizeXZ(newDir, flatDir)) {
        glm::vec3 axis = glm::cross(newDir, flatDir);
        const float axisLength2 = glm::length2(axis);
        if (axisLength2 > 1e-8f) {
          // `acosf(dot)` を避け、angle = atan2(|cross|, dot) を使う。
          // - axis の正規化に |cross| が必要なので、sqrt は1回で済む。
          // - dot が 1 付近の微小角では、数値安定性も良い。
          const float axisLength = glm::sqrt(axisLength2);
          const float dotNF = glm::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f);
          const float flatAngle = atan2f(axisLength, dotNF);
          if (flatAngle > 1e-4f) {
            axis *= (1.0f / axisLength);
            const float rotLimit = globalSpeciesParams[sid].horizontalTorque * dt;
            const float rot = glm::min(flatAngle, rotLimit);
            newDir = approxRotate(newDir, axis, rot);
          }
        }
      }
    }

    // -----------------------------------------------
    // 共通処理: 速度クランプと位置更新
    // -----------------------------------------------
    float finalSpeed =
        glm::clamp(speed, globalSpeciesParams[sid].minSpeed, maxSpeed);

    const glm::vec3 newVelocity = newDir * finalSpeed;
    unit->buf->velocitiesWrite[gIdx] = newVelocity;
    unit->buf->positionsWrite[gIdx] = position + newVelocity * dt;
    obstacle_field::resolvePenetration(unit->buf->positionsWrite[gIdx],
                       unit->buf->velocitiesWrite[gIdx]);
    unit->buf->accelerations[gIdx] = glm::vec3(0.0f);
    unit->buf->predatorInfluences[gIdx] *= 0.7f; // 適度に保持して逃走を継続（保持しすぎると過剰反応になる）
    unit->buf->orientationsWrite[gIdx] = BoidUnit::dirToQuatRollZero(newDir);
    if (unit->buf->stresses[gIdx] > 0.0f) {
      float decayRate = 1.0f;
      unit->buf->stresses[gIdx] -= BoidUnit::easeOut(dt * decayRate);
      if (unit->buf->stresses[gIdx] < 0.0f) {
        unit->buf->stresses[gIdx] = 0.0f;
      }
    }
  }
}

/**
 * ユニット内の Boid または子ノードを基にバウンディングスフィアを計算する。
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - Boid の位置を基に中心と半径を計算。
 * - **中間ノードの場合**:
 *   - 子ノードの中心と半径を基に親ノードの中心と半径を計算。
 *
 * 使用例:
 * - 階層構造内でユニットの境界を計算する際に使用。
 */
void BoidUnit::computeBoundingSphere() {
  if (isBoidUnit()) {
    if (indices.empty())
    {
      simpleDensity = 0.0f;
      BoidTree::instance().setUnitSimpleDensity(id, 0.0f);
      return;
    }

    // 中心を計算
    center = glm::vec3(0.0f);
    for (int gIdx : indices)
      center += buf->positions[gIdx];
    center /= static_cast<float>(indices.size());

    // 半径は「中心からの最大距離」で決める。
    // - leaf は毎フレーム更新されるため、sqrt 回数を最小化したい。
    // - 最大距離ベースなら leaf 内の全個体を確実に包含でき、探索の取りこぼしを避けやすい。
    float maxDistSq = 0.0f;
    for (int gIdx : indices) {
      const float d2 = glm::distance2(center, buf->positions[gIdx]);
      maxDistSq = std::max(maxDistSq, d2);
    }
    radius = (maxDistSq > 0.0f) ? glm::sqrt(maxDistSq) : 0.0f;

    const SpeciesParams *densityParams = nullptr;
    SpeciesParams fallbackParams{};
    if (speciesId >= 0 &&
        speciesId < static_cast<int>(globalSpeciesParams.size())) {
      densityParams = &globalSpeciesParams[speciesId];
    } else {
      fallbackParams.separationRange =
          glm::max(radius, kLeafDensityMinRadius);
      densityParams = &fallbackParams;
    }

    if (buf && densityParams) {
      simpleDensity = computeLeafSimpleDensity(
          indices.data(), indices.size(), *buf, *densityParams);
    } else {
      simpleDensity = 0.0f;
    }
    BoidTree::instance().setUnitSimpleDensity(id, simpleDensity);
  } else {
    if (children.empty())
      return; // 子ノードの中心を計算 - パフォーマンス最適化
    center = glm::vec3(0.0f);
    const size_t childrenSize = children.size();
    const BoidUnit *const *childrenData = children.data();

    for (size_t i = 0; i < childrenSize; ++i) {
      center += childrenData[i]->center;
    }
    center /= static_cast<float>(childrenSize);

    // 子ノード中心までの平均距離 + 子ノード半径 - パフォーマンス最適化
    // NOTE: mean を維持するため sqrt は必要。
    float sum = 0.0f, sum2 = 0.0f;
    for (size_t i = 0; i < childrenSize; ++i) {
      const BoidUnit *child = childrenData[i];
      const float baseD2 = glm::distance2(center, child->center);
      const float baseD = (baseD2 > 0.0f) ? glm::sqrt(baseD2) : 0.0f;
      const float d = baseD + child->radius;
      sum += d;
      sum2 += d * d;
    }
    float mean = sum / static_cast<float>(childrenSize);
    float var = sum2 / static_cast<float>(childrenSize) - mean * mean;
    float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

    radius = mean + 1.0f * stddev;
    simpleDensity = 0.0f;
    BoidTree::instance().setUnitSimpleDensity(id, 0.0f);
  }
}

/** * 他のユニットとの相互作用を計算し、加速度に影響を加える。
 *
 * @param other 他の BoidUnit へのポインタ
 *
 * 処理内容:
 * - **捕食者の場合**: 木構造を使って再帰的に獲物を探索し、捕食圧を適用
 * - **通常の場合**: ユニット間でBoidの相互作用を計算
 *
 * 機能:
 * - 捕食者の逃避反応とストレス計算
 * - ユニット間のBoid相互作用による分離・凝集・整列
 */
void BoidUnit::applyInterUnitInfluence(BoidUnit *other, float dt) {
  if (!other) {
    return;
  }
  auto applyPredatorSweep = [&](BoidUnit *predatorUnit) -> bool {
    if (!predatorUnit || !predatorUnit->isBoidUnit())
      return false;

    int predatorSid = predatorUnit->speciesId;
    if (!globalSpeciesParams[predatorSid].isPredator)
      return false;

    if (predatorUnit->indices.empty())
      return false;

    // 全非捕食者種の警戒距離の最大値（BoidTree 側でキャッシュ済み）
    const float predatorEffectRange = BoidTree::instance().getMaxPredatorAlertRadius();

    auto *soa = predatorUnit->buf;

    // 捕食者の影響は「捕食者個体位置」を基準に球クエリする。
    // - ユニットの center/radius に依存すると列挙漏れが起きるため。
    // - 捕食者数は少ない想定なので、個体ごとのクエリを許容する。
    for (int predatorIdx : predatorUnit->indices) {
      const glm::vec3 predatorPos = soa->positions[predatorIdx];

      // 捕食者スイープは「正確さ」より「それっぽさ」を優先できるため、
      // 周辺列挙を上限つき + ランダムサンプルにして空間探索コストを抑える。
      constexpr std::size_t kPredatorGuaranteedNearest = 8;
        const std::size_t oversampleLimit = kPredatorCacheLimit;

      struct PredatorSweepCandidate {
        int index;
        float distanceSq;
      };
      static thread_local std::vector<PredatorSweepCandidate> preyCandidates;
      preyCandidates.clear();
      if (preyCandidates.capacity() < oversampleLimit) {
        preyCandidates.reserve(oversampleLimit);
      }

      spatial_query::forEachBoidInSphereLimited(
          BoidTree::instance(), predatorPos, predatorEffectRange, oversampleLimit,
          [&](int idxB, const BoidUnit *leafNode) {
            if (!leafNode || leafNode == predatorUnit) {
              return;
            }
            const int sidB = soa->speciesIds[idxB];
            if (sidB < 0 || sidB >= static_cast<int>(globalSpeciesParams.size())) {
              return;
            }
            if (globalSpeciesParams[sidB].isPredator) {
              return;
            }
            const glm::vec3 preyPos = soa->positions[idxB];
            const glm::vec3 toTarget = preyPos - predatorPos;
            const float distSq = glm::dot(toTarget, toTarget);
            preyCandidates.push_back({idxB, distSq});
          });

        const std::size_t candidateCount = preyCandidates.size();
        const std::size_t sampleCount = candidateCount;

      const std::size_t guaranteeCount =
          std::min<std::size_t>(kPredatorGuaranteedNearest, sampleCount);
      if (guaranteeCount > 0 && candidateCount > guaranteeCount) {
        // 最近傍は必ずサンプルに含めて、捕食者直近を空けやすくする。
        std::nth_element(preyCandidates.begin(),
                         preyCandidates.begin() + guaranteeCount,
                         preyCandidates.end(),
                         [](const PredatorSweepCandidate &a,
                            const PredatorSweepCandidate &b) {
                           return a.distanceSq < b.distanceSq;
                         });
      }

      for (std::size_t i = 0; i < sampleCount; ++i) {
        const int idxB = preyCandidates[i].index;
        const int sidB = soa->speciesIds[idxB];
        if (sidB < 0 || sidB >= static_cast<int>(globalSpeciesParams.size())) {
          continue;
        }
        const SpeciesParams &preyParams = globalSpeciesParams[sidB];
        float alertRadius = std::max(preyParams.predatorAlertRadius, 0.0f);
        if (alertRadius <= 0.0f) {
          alertRadius = predatorEffectRange;
        }
        const float alertRadiusSq = alertRadius * alertRadius;

        const glm::vec3 toTarget = soa->positions[idxB] - predatorPos;
        const float d2 = glm::dot(toTarget, toTarget);
        if (d2 >= alertRadiusSq) {
          continue;
        }
        // 同一点（d2=0）の normalize を避けて NaN 混入を防ぐ。
        if (!(d2 > kSafeNormalizeEps2)) {
          continue;
        }

        // 同じ距離に対する sqrt を二重に呼ばない（ホットパス削減）。
        const float dist = std::sqrt(d2);
        const float invDist = (dist > 0.0f) ? (1.0f / dist) : 0.0f;
        const glm::vec3 escapeDir = toTarget * invDist;
        const float normalizedDistance = dist / alertRadius;
        float escapeStrength =
            glm::clamp(1.0f - normalizedDistance, 0.0f, 1.0f);
        escapeStrength =
            escapeStrength * escapeStrength * (3.0f - 2.0f * escapeStrength);

        soa->predatorInfluences[idxB] += escapeDir * escapeStrength * 5.0f;
        soa->predatorThreats[idxB] =
            std::max(soa->predatorThreats[idxB], escapeStrength);

        const float stressLevel =
            glm::clamp(0.4f + escapeStrength * 0.6f, 0.0f, 1.0f);
        soa->stresses[idxB] = std::max(soa->stresses[idxB], stressLevel);
      }
    }

    return true;
  };

  bool predatorHandled = applyPredatorSweep(this);
  predatorHandled = applyPredatorSweep(other) || predatorHandled;

  // 異種ユニット間の通常相互作用は、内部ループで sidA != sidB になり全スキップになる。
  // それでも (indices × other->indices) の走査コストだけが残るため、ここで早期returnして潰す。
  // ※捕食者スイープは上で適用済み。
  if (predatorHandled && !indices.empty() && !other->indices.empty()) {
    if (speciesId >= 0 && other->speciesId >= 0 && speciesId != other->speciesId) {
      return;
    }
  }

  // if (!predatorHandled && frameCount % 6 != 0) {
  //   // 再構築頻度でない場合は通常の相互作用をスキップ
  //   return;
  // }

  // 通常の影響処理（非捕食者または通常時）
  if (!indices.empty() && !other->indices.empty()) {
    for (int idxA : indices) {
      glm::vec3 sumVel = glm::vec3(0.00001f);
      glm::vec3 sumPos = glm::vec3(0.00001f);
      glm::vec3 sep = glm::vec3(0.00001f);
      int cnt = 0;
      int sidA = speciesId;
      if (sidA < 0 || sidA >= static_cast<int>(globalSpeciesParams.size())) {
        continue;
      }

      const SpeciesParams &paramsA = globalSpeciesParams[sidA];
      const float cohesionRange = glm::max(paramsA.cohesionRange, 0.0f);
      const float alignmentRange = glm::max(paramsA.alignmentRange, 0.0f);
      const float interRange = glm::max(cohesionRange, alignmentRange);
      if (interRange <= 1e-4f) {
        continue;
      }
      const float interRangeSq = interRange * interRange;
      for (int idxB : other->indices) {
        // 種族IDチェック: 異なる種族同士では群れ行動を行わない
        int sidB = other->buf->speciesIds[idxB];
        if (sidA != sidB) {
          continue; // 異なる種族とは群れ行動しない
        }

        glm::vec3 diff = buf->positions[idxA] - other->buf->positions[idxB];
        float d2 = glm::dot(diff, diff);

        if (d2 < interRangeSq && d2 > 1e-4f) {
          float d = std::sqrt(d2);
          float w = std::max(0.0f, 1.0f - (d / interRange));

          sumVel += other->buf->velocities[idxB] * w;
          sumPos += other->buf->positions[idxB] * w;
          sep += (diff / (d2 + 1.0f)) * w;
          ++cnt;
        }
      }
      if (cnt > 0) {
        const float longRangeCohesionScale =
            glm::clamp(gSimulationTuning.fastAttractStrength, 0.0f, 1.0f);
        // ストレス状態に応じた段階的動作制御
        float currentStress = buf->stresses[idxA];
        const float predatorThreat =
            glm::clamp(buf->predatorThreats[idxA], 0.0f, 1.0f);
        float cohesionMultiplier = 1.0f;
        float speedMultiplier = 1.0f;

        // 捕食者が近い間は「再結集」より「散開」を優先する。
        // - 凝集/整列を弱めて、捕食者方向へ引っ張られにくくする
        // - 分離を強めて、捕食者突入時に局所的な“穴”ができやすくする
        // predatorThreat は距離由来で即応性が高い（stress より先に上がりやすい）ため優先する。
        // ただし「群れが大きいほど、少し近い程度では焦らない」ように、閾値を近傍数で動かす。
        const float groupSafety = glm::clamp(float(cnt) / 12.0f, 0.0f, 1.0f);
        const float underThreatThreshold = glm::mix(0.08f, 0.22f, groupSafety);
        const bool isUnderPredatorThreat = predatorThreat > underThreatThreshold;

        // 逃走後の再結集フェーズ（脅威が落ちた & 中程度ストレス時）
        if (!isUnderPredatorThreat && currentStress > 0.2f && currentStress < 0.9f) {
          cohesionMultiplier = 5.0f; // 再結集を強める
          speedMultiplier = 1.5f;    // 戻りをキビキビさせる
        }
        // 高ストレス時 or 脅威継続中は逃走優先（凝集力抑制）
        else if (currentStress >= 0.9f || isUnderPredatorThreat) {
          cohesionMultiplier = 0.2f; // 凝集を強く抑えて散開しやすくする
        }

        buf->accelerations[idxA] +=
          (sumVel / float(cnt) - buf->velocities[idxA]) *
          (paramsA.alignment * (isUnderPredatorThreat ? 0.4f : 1.0f));
        buf->accelerations[idxA] +=
          (sumPos / float(cnt) - buf->positions[idxA]) *
          (paramsA.cohesion * cohesionMultiplier * longRangeCohesionScale);
        // 捕食者が近いほど分離を強め、局所密度を素早く下げる。
        const float separationBoost = isUnderPredatorThreat ? (1.0f + predatorThreat * 2.0f) : 1.0f;
        buf->accelerations[idxA] += sep * (paramsA.separation * separationBoost);

        // 再結集中の速度向上を後で適用するため、ストレス情報を保持
        float scaledSpeedBoost =
            1.0f + (speedMultiplier - 1.0f) * longRangeCohesionScale;
        buf->stresses[idxA] =
            glm::max(buf->stresses[idxA], scaledSpeedBoost - 1.0f);
      }
    }
  }
}

/**
 * 再帰的にユニット内の Boid の動きを更新する。
 *
 * @param dt 時間ステップ
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - 各 Boid の加速度を初期化。
 *   - 分離、整列、凝集ルールを適用して加速度を計算。
 *   - 最大速度、最小速度、最大旋回角を考慮して速度を更新。
 *   - Boid の位置を更新。
 * - **中間ノードの場合**:
 *   - 再帰的に子ノードを処理。
 *   - 子ノードの結果（中心、平均速度など）を親ノードに伝搬。
 *
 * 使用例:
 * - 階層構造内で各ユニットの Boid の動きを更新する際に使用。
 */
void BoidUnit::updateRecursive(float dt) {
  frameCount++;

  // 無限再帰防止: 簡単なカウンター方式
  static int callCount = 0;
  callCount++;

  if (callCount > 10000) {
    callCount = 0; // リセット
    return;        // 過度な再帰を防止
  }

  // ----------------------------------------------
  // 並列化の基本方針
  // ----------------------------------------------
  // leaf をチャンクにまとめ、少数タスクで処理する。
  // - タスク/ future の発行が細かすぎると、待機・同期のオーバーヘッドが支配的になりやすい。
  auto &pool = getThreadPool();
  static std::vector<std::future<void>> asyncTasks;
  asyncTasks.clear();
  asyncTasks.reserve(16);

  const std::size_t hw = std::max(1u, std::thread::hardware_concurrency());
  // タスク数は CPU スレッド数に比例させるが、過剰分割は逆効果なので上限を付ける。
  std::size_t maxTasks = std::min<std::size_t>(hw, 8);

#ifdef __EMSCRIPTEN__
  // main browser thread で Atomics.wait/futex wait すると待ち時間が支配的になりやすい。
  // ここではメインスレッド上での過剰並列化だけを抑え、
  // 「完全逐次化」による性能低下を避ける。
  if (emscripten_is_main_browser_thread()) {
    maxTasks = std::min<std::size_t>(maxTasks, 4);
  }
#endif

  auto runParallelRanges = [&](std::size_t total, auto &&fn) {
    using Fn = std::decay_t<decltype(fn)>;
    Fn fnCopy = std::forward<decltype(fn)>(fn);
    if (total == 0) {
      return;
    }
    const std::size_t taskCount = std::min<std::size_t>(maxTasks, total);
    const std::size_t chunk = (total + taskCount - 1) / taskCount;

    asyncTasks.clear();
    // 1チャンク分は現在スレッドで実行し、残りだけ enqueue してオーバーヘッドを抑える。
    for (std::size_t t = 1; t < taskCount; ++t) {
      const std::size_t begin = t * chunk;
      if (begin >= total) {
        break;
      }
      const std::size_t end = std::min(total, begin + chunk);
      asyncTasks.emplace_back(
          pool.enqueue([=, &fnCopy] { fnCopy(begin, end); }));
    }

    fnCopy(0, std::min(total, chunk));

    for (auto &f : asyncTasks) {
      f.get();
    }
    asyncTasks.clear();
  };

  // ----------------------------------------------
  // 第一段階: acceleration を計算
  // ----------------------------------------------
  // 毎フレームの確保/解放を避けるために static で再利用する。
  // updateRecursive 自体がスレッドセーフでない（callCount が static）のため、ここも同様に割り切る。
  static std::vector<BoidUnit *> leafUnits;
  static std::vector<BoidUnit *> predatorLeafUnits;
  leafUnits.clear();
  predatorLeafUnits.clear();
  if (leafUnits.capacity() < 512) {
    leafUnits.reserve(512);
  }
  if (predatorLeafUnits.capacity() < 8) {
    predatorLeafUnits.reserve(8);
  }

  std::stack<BoidUnit *, std::vector<BoidUnit *>> stack;
  stack.push(this);

  int firstStageOperations = 0;
  while (!stack.empty() && firstStageOperations < 10000) {
    firstStageOperations++;
    BoidUnit *current = stack.top();
    stack.pop();
    if (!current) {
      continue;
    }

    if (current->isBoidUnit()) {
      leafUnits.push_back(current);
      const int sid = current->speciesId;
      if (sid >= 0 && sid < static_cast<int>(globalSpeciesParams.size()) &&
          globalSpeciesParams[sid].isPredator) {
        predatorLeafUnits.push_back(current);
      }
      continue;
    }

    // 内部ノードは子を走査し、バウンディング球だけ更新しておく。
    current->computeBoundingSphere();

    const size_t childrenSize = current->children.size();
    BoidUnit **childrenData = current->children.data();
    for (size_t i = 0; i < childrenSize; ++i) {
      stack.push(childrenData[i]);
    }

    // 子ユニット間の相互作用は、
    // - 捕食者ペアは別ルートで処理（ペアごとに実行すると重い & 冗長）
    // - 非捕食者は距離カリングして必要なものだけ実行
    if (childrenSize > 1) {
      for (size_t a = 0; a < childrenSize; ++a) {
        BoidUnit *ua = childrenData[a];
        if (!ua || ua->indices.empty()) {
          continue;
        }
        const int sidA = ua->speciesId;
        const bool predatorA =
            sidA >= 0 && sidA < static_cast<int>(globalSpeciesParams.size()) &&
            globalSpeciesParams[sidA].isPredator;
        if (predatorA) {
          continue;
        }
        if (sidA < 0 || sidA >= static_cast<int>(globalSpeciesParams.size())) {
          continue;
        }
        const float interRange =
            glm::max(globalSpeciesParams[sidA].cohesionRange,
                     globalSpeciesParams[sidA].alignmentRange);

        for (size_t b = a + 1; b < childrenSize; ++b) {
          BoidUnit *ub = childrenData[b];
          if (!ub || ub->indices.empty()) {
            continue;
          }
          const int sidB = ub->speciesId;
          const bool predatorB =
              sidB >= 0 && sidB < static_cast<int>(globalSpeciesParams.size()) &&
              globalSpeciesParams[sidB].isPredator;
          if (predatorB) {
            continue;
          }

          // 非捕食者同士の群れ行動は同種のみ（applyInterUnitInfluence と整合）
          if (sidA != sidB) {
            continue;
          }

          // まずユニット球で高速カリング。
          const glm::vec3 d = ua->center - ub->center;
          const float maxDist = glm::max(interRange, 0.0f) + ua->radius + ub->radius;
          if (glm::dot(d, d) > maxDist * maxDist) {
            continue;
          }

          // 既存ロジックに合わせ、片方向（a -> b）だけ適用する。
          ua->applyInterUnitInfluence(ub);
        }
      }
    }
  }

  // leaf の相互作用（高コスト）をチャンク並列で実行
  runParallelRanges(leafUnits.size(), [&](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      BoidUnit *unit = leafUnits[i];
      if (unit) {
        unit->computeBoidInteraction(dt);
      }
    }
  });

  // 捕食者の影響は「ペアごと」だと冗長なので、捕食者ユニットごとに一回だけ実行。
  // 捕食者数は少数が前提のため、ここは敢えて逐次実行してデータ競合も避ける。
  for (BoidUnit *pred : predatorLeafUnits) {
    if (pred) {
      // predator sweep は相手ユニットと無関係に SpatialIndex で獲物を列挙する。
      // self-self を渡すと通常相互作用まで走り得るため、内部ノード（this）を相手にして
      // スイープだけを確実に実行する。
      pred->applyInterUnitInfluence(this, dt);
    }
  }

  // ----------------------------------------------
  // 第二段階: 位置と速度を更新
  // ----------------------------------------------
  runParallelRanges(leafUnits.size(), [&](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      BoidUnit *unit = leafUnits[i];
      if (unit) {
        updateLeafKinematics(unit, dt);
      }
    }
  });

  // 関数終了時にカウンターをリセット
  callCount--;
}
inline float BoidUnit::easeOut(float t) {
  // イージング関数 (ease-out)
  return t * t * (3.0f - 2.0f * t);
}
inline glm::quat BoidUnit::dirToQuatRollZero(const glm::vec3 &forward) {
  // forward がゼロ長だと normalize が NaN を返し、以降の姿勢が破綻する。
  // kinematics 側で極力防いでいるが、保険としてここでもゼロ長を避ける。
  glm::vec3 f = safeNormalizeOr(forward, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::vec3 up(0.0f, 1.0f, 0.0f);
  if (fabsf(glm::dot(f, up)) > 0.99f) { // 平行回避
    up = glm::vec3(1.0f, 0.0f, 0.0f);   // フォールバック
  }
  glm::vec3 right = glm::cross(up, f);
  right = safeNormalizeOr(right, glm::vec3(1.0f, 0.0f, 0.0f));
  up = glm::cross(f, right); // 直交基底
  glm::mat3 R(right, up, f); // 列順：X,Y,Z
  return glm::quat_cast(R);  // 正規化済 quat
}

/**
 * ユニット内の各Boidについて近傍Boidとの相互作用を計算し、加速度を更新する。
 *
 * 機能:
 * - 動的近傍管理（cohesionMemories, activeNeighbors）
 * - 群れの3要素（分離・整列・凝集）に基づく加速度計算
 * - Fast-start吸引制御による群れの縁での強制凝集
 * - 捕食者の追跡ターゲット選択と更新
 */
void BoidUnit::computeBoidInteraction(float dt) {
  // 空間インデックス向けの境界情報を毎フレーム更新
  computeBoundingSphere();

  const int globalFrame = BoidTree::instance().frameCount;

  glm::vec3 separation;
  glm::vec3 alignment;
  glm::vec3 cohesion;
  int gIdx = 0;
  glm::vec3 pos;
  glm::vec3 vel;
  int sid = -1;
  // 軽量なランダム数生成（WASMでmt19937が使えないため）
  static uint32_t rng_state = 1;
  auto simple_rand = [&]() -> uint32_t {
    rng_state = rng_state * 1103515245 + 12345;
    return (rng_state >> 16) & 0x7fff;
  };
  auto rand_range = [&](int max_val) -> int { return simple_rand() % max_val; };

  // 近傍記憶の期限(τ)を個体ごとに少しずらすための軽量ハッシュ。
  // 同じτで一斉に記憶が切れると、近傍集合が同時に入れ替わり
  // “群れ全体が急に同じ向きへ旋回する”挙動が出やすい。
  auto hash01 = [&](uint32_t x) -> float {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    // 下位24bitを0..1へ（WASMでも十分軽量）
    return float(x & 0x00ffffffU) * (1.0f / 16777215.0f);
  };

  // -----------------------------------------------
  // 事前計算しておく定数／準備
  // -----------------------------------------------
  // 非ゼロ判定用イプシロン
  constexpr float EPS =
      1e-8f; // 候補距離を入れてソート/部分ソートするための領域
  // thread_local で確保コストを抑えつつ再利用。上限超過時は後段で縮小する。
  static thread_local std::vector<std::pair<float, int>> candidates;
  static thread_local std::vector<int> predatorTargetCandidates;
  // SpatialIndex（球クエリ）から「ユニット外」の近傍を補うための一時バッファ。
  // 既存の activeNeighbors/cohesionMemories は leaf 内 index 前提なので、
  // 外部近傍はフレーム内の加速度計算にのみ利用し、メモリ構造は崩さない。
  static thread_local std::vector<int> externalNeighbors;
  if (candidates.capacity() < indices.size()) {
    candidates.reserve(indices.size());
  }

  const int leafSpeciesId = speciesId;
  const float leafDensityValue = simpleDensity;

  // -----------------------------------------------
  // 種族ごとに一定な値は先に計算して使い回す
  // -----------------------------------------------
  // NOTE:
  // - computeBoidInteraction は N 個体ぶん繰り返すため、三角関数や sqrt を1回でも減らすと効く。
  // - leaf は同種族で構成される前提（異種 merge は禁止済み）なので、ここでホイストする。
  const SpeciesParams &leafParams = globalSpeciesParams[leafSpeciesId];
  const float leafViewRange = glm::max(leafParams.cohesionRange, 0.0f);
  const float leafViewRangeSq = leafViewRange * leafViewRange;
  const float halfFovRad = glm::radians(leafParams.fieldOfViewDeg * 0.5f);
  const float leafCosHalfFovSq = std::cos(halfFovRad) * std::cos(halfFovRad);

  // -----------------------------------------------
  // 各 Boid（leafノード内）ごとの反復
  // -----------------------------------------------
  for (size_t index = 0; index < indices.size(); ++index) {
    // -------------------------------------------------------
    // 1. 初期化フェーズ
    //    - 加速度計算用に separation/alignment/cohesion をリセット
    //    - 対象 Boid のグローバルインデックスと位置・速度を取得
    // -------------------------------------------------------
    separation = glm::vec3(0.00001f);
    alignment = glm::vec3(0.00001f);
    cohesion = glm::vec3(0.00001f);
    sid = speciesId;
    gIdx = indices[index];
    pos = buf->positions[gIdx];
    vel = buf->velocities[gIdx];
    const SpeciesParams &selfParams = globalSpeciesParams[sid];
    if (!selfParams.isPredator) {
      buf->predatorRestTimers[gIdx] = 0.0f;
      buf->predatorChaseTimers[gIdx] = 0.0f;
    }
    // 外部近傍計算でも使うため、自己形状の絶対値/半径を先に確定しておく。
    const float selfHeadAbs = std::abs(selfParams.bodyHeadLength);
    const float selfTailAbs = std::abs(selfParams.bodyTailLength);
    const float selfRadiusAbs = std::max(selfParams.bodyRadius, 0.0f);
    const float baseCohesionStrength = glm::max(selfParams.cohesion, 0.0f);
    glm::vec3 longTermCohesion(0.0f);
    // 近接回避は距離ベースの軽量反発で扱う。
    // - 近傍ごとの高コスト計算や、相手状態の更新は避けて並列性を保つ。
    float threatLevel = glm::clamp(buf->predatorThreats[gIdx], 0.0f, 1.0f);
    const float viewRangeSq = leafViewRangeSq;

    // FastAttract 機能を廃止したため、状態は毎フレーム明示的にリセットする。
    // バッファを持ち越すと「過去のON状態」が残って挙動が不安定になり得る。
    buf->isAttracting[gIdx] = 0;
    buf->attractTimers[gIdx] = 0.0f;

    // 小クラスターより更に上位の「群れ（大クラスター）」中心を使う。
    // - 小クラスターはフレーム間でスイッチしやすく、群れ全体の中心としては不安定になりがち
    // - 大クラスターは 10秒EMA で追跡されるため、長期の回転/凝集の基準点に向く
    glm::vec3 schoolCenterDir(0.0f);
    float schoolRadius = 1.0f;
    float schoolWeight = 0.0f;
    bool hasSchoolCenterDir = false;

    // 大クラスタ中心（school cluster）を「ほぼ同一点」の重複を避けつつ選ぶ。
    // - 半径が重なっていてもOK（混ざる塊は許容）
    // - ただし中心がほぼ同じ座標だと候補を増やしても意味がないため除外する
    // ※ユーザー指定: minAbs = 2
    constexpr int kMaxSchoolCoreCandidates = 3;
    constexpr float kSchoolCoreMinAbs = 2.0f;
    constexpr float kSchoolCoreMinAbsSq = kSchoolCoreMinAbs * kSchoolCoreMinAbs;
    struct SchoolCoreCandidate {
      glm::vec3 center;
      float radius;
      float weight;
    };

    if (sid >= 0) {
      const auto *schools = BoidTree::instance().getSpeciesSchoolClusters(sid);
      if (schools) {
        std::array<SchoolCoreCandidate, kMaxSchoolCoreCandidates> coreCandidates{};
        int coreCandidateCount = 0;

        // 1) 種族の school clusters から、中心が近すぎないものを最大K個まで確保（重み優先）
        for (const auto &school : *schools) {
          if (!school.active || school.weight < 0.25f) {
            continue;
          }

          bool tooClose = false;
          for (int i = 0; i < coreCandidateCount; ++i) {
            const glm::vec3 d = school.center - coreCandidates[i].center;
            if (glm::dot(d, d) < kSchoolCoreMinAbsSq) {
              tooClose = true;
              break;
            }
          }
          if (tooClose) {
            continue;
          }

          const SchoolCoreCandidate candidate{
              school.center, glm::max(school.radius, 1.0f), school.weight};
          if (coreCandidateCount < kMaxSchoolCoreCandidates) {
            coreCandidates[coreCandidateCount++] = candidate;
          } else {
            // 既に満杯なら、最も軽い候補を置き換える（重いコアを優先して残す）
            int minIndex = 0;
            float minWeight = coreCandidates[0].weight;
            for (int i = 1; i < coreCandidateCount; ++i) {
              if (coreCandidates[i].weight < minWeight) {
                minWeight = coreCandidates[i].weight;
                minIndex = i;
              }
            }
            if (candidate.weight > minWeight) {
              coreCandidates[minIndex] = candidate;
            }
          }
        }

        // 2) 候補コアの中から、この個体に最も近いものを選ぶ
        float bestDistSq = std::numeric_limits<float>::max();
        for (int i = 0; i < coreCandidateCount; ++i) {
          const glm::vec3 diff = coreCandidates[i].center - pos;
          const float distSq = glm::dot(diff, diff);
          if (distSq < bestDistSq) {
            bestDistSq = distSq;
            schoolCenterDir = diff;
            schoolRadius = coreCandidates[i].radius;
            schoolWeight = coreCandidates[i].weight;
            hasSchoolCenterDir = true;
          }
        }
      }
    }
    const float cosHalfFovSq = leafCosHalfFovSq;

    candidates.clear();
    if (globalSpeciesParams[sid].isPredator) {
      // 毎フレーム更新される簡易フェーズ用のベクトルをリセット
      buf->predatorApproachDirs[gIdx] = glm::vec3(0.0f);
      // 捕食者の追跡/休憩サイクルを管理する
      int &tgtIdx = buf->predatorTargetIndices[gIdx];
      float &tgtTime = buf->predatorTargetTimers[gIdx];
      float &restTimer = buf->predatorRestTimers[gIdx];
      float &chaseTimer = buf->predatorChaseTimers[gIdx];
      if (restTimer > 0.0f) {
        restTimer = glm::max(restTimer - dt, 0.0f);
      }
      const bool predatorOnBreak = restTimer > 0.0f;

      if (predatorOnBreak) {
        tgtIdx = -1;
        tgtTime = 0.0f;
        chaseTimer = 0.0f;
      } else {
        // スレッドごとにバッファを共有して動的確保コストを抑える
        // 毎フレームクールダウンを減算（ターゲット消失時も進行）
        tgtTime -= dt;
        // ターゲットは「時間切れ」だけでなく「範囲外」「不正インデックス」でも再選択する。
        // これが無いと、獲物が遠方へ逃げた後も tau が切れるまで延々追い続け、
        // “追う対象が固定されているように見える”挙動になりやすい。
        const int boidCount = static_cast<int>(buf->positions.size());
        bool targetInvalid = (tgtIdx < 0) || (tgtIdx >= boidCount) || (tgtTime <= 0.0f);
        if (!targetInvalid) {
          const int tgtSpecies = buf->speciesIds[tgtIdx];
          if (tgtSpecies < 0 ||
              tgtSpecies >= static_cast<int>(globalSpeciesParams.size()) ||
              globalSpeciesParams[tgtSpecies].isPredator) {
            targetInvalid = true;
          }
        }
        if (!targetInvalid) {
          const glm::vec3 diff = buf->positions[tgtIdx] - pos;
          const float distSq = glm::dot(diff, diff);
          // 旧来ロジックの探索半径をそのまま「追跡維持の上限」としても使う。
          // 逃げ切られたら素直に別ターゲットへ切り替える。
          const float targetSearchRadius = 100.0f;
          const float maxChaseDistSq = targetSearchRadius * targetSearchRadius;
          if (distSq > maxChaseDistSq) {
            targetInvalid = true;
          }
        }
        if (targetInvalid) {
          tgtIdx = -1;
          predatorTargetCandidates.clear();
          if (predatorTargetCandidates.capacity() < kPredatorCacheLimit) {
            predatorTargetCandidates.reserve(kPredatorCacheLimit);
          }
          const float targetSearchRadius = 100.0f; // 旧来ロジックの探索半径を維持

            // SpatialIndex を通じて周辺の非捕食者を直接収集し、
            // ついでに「獲物密集の中心（簡易）」を作って接近方向に使う。
          constexpr std::size_t kPredatorCandidateLimit = 256;
            glm::vec3 preyCenterSum(0.0f);
            int preyCenterCount = 0;
          spatial_query::forEachBoidInSphereLimited(
              BoidTree::instance(), pos, targetSearchRadius,
              kPredatorCandidateLimit,
              [&](int candidateIdx, const BoidUnit *leafNode) {
                if (leafNode == this || candidateIdx == gIdx) {
                  return;
                }
                const int candidateSpecies = buf->speciesIds[candidateIdx];
                if (globalSpeciesParams[candidateSpecies].isPredator) {
                  return;
                }
                predatorTargetCandidates.push_back(candidateIdx);
                preyCenterSum += buf->positions[candidateIdx];
                ++preyCenterCount;
              });

          if (preyCenterCount > 0) {
            const glm::vec3 preyCenter =
                preyCenterSum * (1.0f / static_cast<float>(preyCenterCount));
            buf->predatorApproachDirs[gIdx] = preyCenter - pos;
          }

          if (!predatorTargetCandidates.empty()) {
            // 「群れへ突っ込む」→「個体追跡」に見せるため、
            // いきなり追跡を開始せず、ある程度中心に近づいてからターゲットを確定する。
            constexpr float kEngageRadius = 60.0f;
            const glm::vec3 approach = buf->predatorApproachDirs[gIdx];
            const float approachDistSq = glm::dot(approach, approach);
            if (approachDistSq <= kEngageRadius * kEngageRadius) {
              const int pick = rand_range(
                  static_cast<int>(predatorTargetCandidates.size()));
              tgtIdx = predatorTargetCandidates[pick];
              tgtTime = globalSpeciesParams[sid].tau;
            } else {
              // まだ遠いので接近フェーズを継続。次フレームも候補取得するために 0 にしておく。
              tgtIdx = -1;
              tgtTime = 0.0f;
            }
          } else {
            // ターゲット候補がいない場合は即再試行できるようリセット
            tgtTime = 0.0f;
          }
        }

        if (tgtIdx >= 0) {
          chaseTimer += dt;

          // “食べた/捕まえた”の近似：一定距離まで近づいたら即終了して離脱に移る。
          // 厳密な衝突判定ではなく、見た目のフェーズ切替を優先。
          {
            const glm::vec3 diff = buf->positions[tgtIdx] - pos;
            const float distSq = glm::dot(diff, diff);
            constexpr float kCaptureDist = 2.0f;
            if (distSq <= kCaptureDist * kCaptureDist) {
              const float restBase = glm::clamp(
                  globalSpeciesParams[sid].tau * kPredatorRestScale,
                  kPredatorRestMin, kPredatorRestMax);
              const float restNoise = hash01(
                  uint32_t(gIdx) * 1664525u +
                  uint32_t(globalFrame) * 1013904223u);
              restTimer = restBase * glm::mix(0.7f, 1.3f, restNoise);
              chaseTimer = 0.0f;
              // 離脱方向を「獲物中心から離れる」へ寄せる
              buf->predatorDisengageDirs[gIdx] = -diff;
              tgtIdx = -1;
              tgtTime = 0.0f;
            }
          }

          const float chaseLimit = glm::clamp(
              globalSpeciesParams[sid].tau * kPredatorChaseScale,
              kPredatorChaseMin, kPredatorChaseMax);
          if (chaseTimer >= chaseLimit) {
            const float restBase = glm::clamp(
                globalSpeciesParams[sid].tau * kPredatorRestScale,
                kPredatorRestMin, kPredatorRestMax);
            const float restNoise = hash01(
                uint32_t(gIdx) * 1664525u + uint32_t(globalFrame) * 1013904223u);
            restTimer = restBase * glm::mix(0.7f, 1.3f, restNoise);
            chaseTimer = 0.0f;
            // 離脱方向は「現在の獲物方向と逆」にして、群れの外へ抜ける見た目を作る
            {
              const glm::vec3 diff = buf->positions[tgtIdx] - pos;
              buf->predatorDisengageDirs[gIdx] = -diff;
            }
            tgtIdx = -1;
            tgtTime = 0.0f;
          }
        } else {
          // 追跡していない時間は徐々に疲労を解消して次の追跡を延長しやすくする
          chaseTimer = glm::max(chaseTimer - dt * 0.35f, 0.0f);
        }
      }
    }

    // -------------------------------------------------------
    // 2. 時間更新と古くなった記憶の無効化
    //    - cohesionMemories[i] > 0 のものは時間を加算
    //    - tau を超えたら 0 に戻してビットをクリア
    //    - activeCount には有効な隣接 Boid 数を数える
    // -------------------------------------------------------
    int activeCount = 0;

    // SOA バッファの境界確認
    if (gIdx >= static_cast<int>(buf->boidCohesionMemories.size()) ||
        gIdx >= static_cast<int>(buf->boidActiveNeighbors.size())) {
      // 境界を超えた場合はスキップ
      continue;
    }

    auto &cohesionMemories =
        buf->boidCohesionMemories[gIdx]; // dt累積（-1.0fで未使用）
    auto &activeNeighbors =
        buf->boidActiveNeighbors[gIdx]; // 使用中slotのインデックス

    // 近傍キャッシュは固定スロット数で管理するため、maxNeighbors はその範囲にクランプする。
    // - 設定値だけが大きいと「近傍不足」判定が常態化し、補助ロジックが過剰に走りやすい。
    const int neighborSlotLimit = static_cast<int>(
      std::min<std::size_t>(cohesionMemories.size(), SoABuffers::NeighborSlotCount));
    const int maxNeighbors = glm::clamp(globalSpeciesParams[sid].maxNeighbors, 0,
                      neighborSlotLimit);

    // cohesionMemories サイズが indices.size() と一致しない場合があるため確認
    size_t maxMemoryIndex =
      std::min(indices.size(), static_cast<std::size_t>(neighborSlotLimit));

    for (size_t i = 0; i < maxMemoryIndex; ++i) {
      if (cohesionMemories[i] > 0.0f) {
        cohesionMemories[i] += dt;

        // 記憶の寿命は完全に一律にせず、個体・スロットごとに微小ジッタを入れる。
        // これにより近傍の入れ替わりが分散し、全体の急旋回を抑える。
        const float baseTau = globalSpeciesParams[sid].tau;
        const float tauJitter =
            baseTau * (0.85f + 0.30f * hash01(uint32_t(gIdx) * 1664525u + uint32_t(i) * 1013904223u));
        if (cohesionMemories[i] > tauJitter) {
          cohesionMemories[i] = 0.0f;
          activeNeighbors.reset(i);
        } else {
          activeCount++;
        }
      }
    } // -------------------------------------------------------
    // 3. 未登録Boidで最も近い（距離かつ視界内）ものを探索
    //    - activeCount < maxNeighbors のときだけ実行
    //    - 距離判定: distSq < viewRangeSq
    //    - 視界判定: normalized(diff)·normalized(vel) >= cosHalfFov
    // -------------------------------------------------------
    // 速度方向は近傍(FOV)判定や、ユニット外近傍の取得でも参照する。
    // スコープを揃えて、無駄な再計算も避ける。
    const float velLen2 = glm::length2(vel);
    const bool hasVel = (velLen2 > EPS);
    glm::vec3 forward(0.0f);
    if (hasVel) {
      forward = vel * (1.0f / glm::sqrt(velLen2));
    }

    if (activeCount < maxNeighbors) {

      for (size_t i = 0; i < indices.size(); ++i) {
        if (i == index)
          continue;
        // 境界確認を追加
        if (i >= cohesionMemories.size())
          continue;
        if (activeNeighbors.test(i) || cohesionMemories[i] > 0.0f)
          continue;

        int gNeighbor = indices[i];
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (distSq >= viewRangeSq)
          continue; // 速度ゼロでなければ視界内かどうかを確認（sqrt を避ける版）
        if (hasVel) {
          // FOV判定: diffDot >= cosHalfFov * |diff|
          // sqrt を避けるために二乗比較へ変形する。
          const float diffDot = glm::dot(forward, diff);
          if (diffDot <= 0.0f) {
            continue;
          }
          if (diffDot * diffDot < cosHalfFovSq * distSq) {
            continue;
          }
        }
        candidates.emplace_back(distSq, (int)i);
      }
    } // -------------------------------------------------------
    // 4. 候補リストから最も近いものを選んで登録
    //    - toAdd = maxNeighbors - activeCount
    //    - 部分ソート (nth_element) で上位toAdd件を取得
    // -------------------------------------------------------
    int toAdd = maxNeighbors - activeCount;
    if (toAdd < 0) {
      toAdd = 0;
    }
    if (toAdd > 0 && !candidates.empty()) {
      if ((int)candidates.size() > toAdd) {
        std::nth_element(candidates.begin(), candidates.begin() + toAdd,
                         candidates.end(),
                         [](auto &a, auto &b) { return a.first < b.first; });
        for (int k = 0; k < toAdd; ++k) {
          int idx2 = candidates[k].second;
          // 境界確認を追加
          if (idx2 >= 0 && idx2 < static_cast<int>(cohesionMemories.size())) {
            cohesionMemories[idx2] = dt;
            activeNeighbors.set(idx2);
          }
        }
      } else {
        for (auto &pr : candidates) {
          int idx2 = pr.second;
          // 境界確認を追加
          if (idx2 >= 0 && idx2 < static_cast<int>(cohesionMemories.size())) {
            cohesionMemories[idx2] = dt;
            activeNeighbors.set(idx2);
          }
        }
      }
    } // -------------------------------------------------------
    // 5. 有効なBoidだけで最終的な加速度を計算
    //    - activeNeighbors.test(i)==true のもののみ
    //    - 分離・凝集・整列の力を合算
    //    - 回転トルクで alignment方向へ向ける補正を実行
    // -------------------------------------------------------
    // -------------------------------------------------------
    // 5-A. 近傍不足 Fast-start 吸引制御 (φᵢ = |Lᵢ| / Nu)
    //      - φᵢ < 1 なら吸引 ON, タイマー τ をリセット
    //      - φᵢ = 1 なら τ カウントダウン → 0 で OFF
    // -------------------------------------------------------
    const size_t neighborSlots =
      std::min(indices.size(), static_cast<std::size_t>(neighborSlotLimit));
    int neighborCount = 0;
    for (size_t i = 0; i < neighborSlots; ++i) {
      if (!activeNeighbors.test(i)) {
        continue;
      }
      if (cohesionMemories[i] <= 0.0f) {
        continue;
      }
      ++neighborCount;
    }

    // -------------------------------------------------------
    // 4.5. SpatialIndex（球クエリ）からユニット外の近傍を補う
    // -------------------------------------------------------
    // 目的: ユニット境界で近傍が途切れて「チンダル現象」的な散り方になるのを抑える。
    // 注意: 完全に正しい kNN を保証するものではない（球内列挙+上限）。
    // 性能: 毎フレーム全 Boid で走らせないため、stride で間引き。
    externalNeighbors.clear();
    // ユニット外近傍の補完は、切れ目の対策として有効だがコストが高い。
    // 常時走らせず「かなり近傍が足りない」時だけ、かつフレーム間引きを強める。
    const bool wantsExternal = (neighborCount * 2 < maxNeighbors);
    constexpr int kExternalNeighborStride = 8;
    const bool externalThrottleHit =
        (((globalFrame + gIdx) % kExternalNeighborStride) == 0);
    const bool lostBoid = (neighborCount == 0);
    if (wantsExternal && (externalThrottleHit || lostBoid)) {
      // cohesionRange を基本に、最低限 separationRange も含む半径にする。
      const float queryRadius = glm::max(selfParams.cohesionRange,
                                         glm::max(selfParams.separationRange, 0.0f));
      if (queryRadius > 1e-4f) {
        // 近傍が足りない分だけ集めたいが、球内が多いケースに備えて上限を置く。
        const int desired = glm::clamp(maxNeighbors - neighborCount, 0, 16);
        const int hardLimit = glm::clamp(maxNeighbors + 8, 8, 24);
        externalNeighbors.reserve(static_cast<std::size_t>(hardLimit));

        spatial_query::forEachBoidInSphereLimited(
            BoidTree::instance(), pos, queryRadius,
            static_cast<std::size_t>(hardLimit),
            [&](int candidateIdx, const BoidUnit *leafNode) {
              // leaf 内候補は既存の activeNeighbors で扱うので除外。
              if (!leafNode || leafNode == this) {
                return;
              }
              if (candidateIdx == gIdx) {
                return;
              }
              const int candidateSid = buf->speciesIds[candidateIdx];
              if (candidateSid != sid) {
                return;
              }

              const glm::vec3 diff = buf->positions[candidateIdx] - pos;
              const float distSq = glm::dot(diff, diff);
              if (!(distSq > EPS && distSq < viewRangeSq)) {
                return;
              }

              // 視界判定（速度がゼロに近い場合は無条件許可）
              if (hasVel) {
                const float diffDot = glm::dot(forward, diff);
                if (diffDot <= 0.0f) {
                  return;
                }
                if (diffDot * diffDot < cosHalfFovSq * distSq) {
                  return;
                }
              }

              // 重複チェック（外部は最大でも数十なので線形で十分）
              for (int existing : externalNeighbors) {
                if (existing == candidateIdx) {
                  return;
                }
              }

              externalNeighbors.push_back(candidateIdx);
            });
      }
    }

    const int externalNeighborCount = static_cast<int>(externalNeighbors.size());
    const int totalNeighborCount = neighborCount + externalNeighborCount;

    if (totalNeighborCount == 0) {
      // 近傍が完全に途切れた場合は、余計な補助機構で加速させずに保守的に振る舞う。
      // ただし大クラスター中心が取れている場合のみ、弱い誘導で群れへ復帰させる。
      if (hasSchoolCenterDir) {
        float clusterLen2 = glm::length2(schoolCenterDir);
        if (clusterLen2 > EPS) {
          glm::vec3 clusterDir = schoolCenterDir *
                                 (1.0f / glm::sqrt(clusterLen2));
          const float clusterPull = baseCohesionStrength *
                                    glm::clamp(schoolWeight * 0.001f, 0.15f,
                                               0.6f);
          buf->accelerations[gIdx] += clusterDir * clusterPull;
        }
      }

      continue;
    }

    // phi は「近傍がどれだけ充足しているか」を表す指標。
    // 近傍数が不足している状況を検出し、凝集の補助などに利用する。
    const int phiDenom = std::max(
        1, std::min(maxNeighbors,
                    static_cast<int>(neighborSlots) + externalNeighborCount));
    float phi = float(totalNeighborCount) / float(phiDenom);

    if (totalNeighborCount > 0) {
      // 近傍のストレスを集約し、逃走の波を伝搬させる。
      float selfStress = buf->stresses[gIdx];
      float stressGainSum = 0.0f;
      float stressWeightSum = 0.0f;
      const float propagationRadius =
          glm::max(globalSpeciesParams[sid].cohesionRange, 1.0f);
      const float propagationBlend = 0.7f; // dt を掛けて応答速度を調整

      // predatorThreats(0〜1) も近傍から伝搬させる。
      // 目的:
      // - 捕食者が群れに突入した際に、直接見ていない個体も「周囲が逃げている」ことで素早く散開する
      // - 逃避が局所だけで終わって「突っ込む/曲がれない」状態になりにくくする
      // 注意:
      // - 値域は 0〜1 のまま（正規化スカラー）
      // - 伝搬は距離で減衰し、dt でブレンドしてフレームレートに依存しにくくする
      float selfThreat = glm::clamp(buf->predatorThreats[gIdx], 0.0f, 1.0f);
      float threatGainSum = 0.0f;
      float threatWeightSum = 0.0f;
      const float threatPropagationRadius = propagationRadius * 1.25f;
      const float threatPropagationBlend = 0.9f;
      // 伝搬は hop ごとに減衰させないと、捕食者が遠い場所まで 0〜1 が均一に広がりやすい。
      // ここでは「近傍から受け取る脅威」を少し減衰させ、距離や hop を経るほど弱まるようにする。
      const float threatTransmission = 0.82f;

      glm::vec3 sumSep = glm::vec3(0.0f);
      glm::vec3 sumAlign = glm::vec3(0.0f);
      glm::vec3 sumCohDir = glm::vec3(0.0f); // 相対ベクトルで凝集を計算
      float wCohSum = 0.0f;                  // 凝集重みの総和
      // 近傍から推定した「局所中心」方向。ミリング（回転）用に再利用する。
      glm::vec3 localCenterDir = glm::vec3(0.0f);
      bool hasLocalCenterDir = false;
      const float cohesionRange = globalSpeciesParams[sid].cohesionRange;
      const float cohesionRangeSq = cohesionRange * cohesionRange;
      int closeNeighborCount = 0;
      int aggregatedNeighborCount = 0;
      // wSep(近さ)の平均で「詰まり具合」を取る。近傍数(maxNeighbors)が飽和しても
      // 密集/外縁の差が出るので、減速や凝集抑制のトリガとして使える。
      float crowdingWeightSum = 0.0f;
      // 「分離が効くレベルに近い個体が複数いる」= 本当に詰まっている、を検出する。
      // wSep は separationRange で正規化済みなので、閾値判定で個体数を数える。
      int crowdCloseCount = 0;

      // 近接反発（めり込み）を検出した場合だけ、速度を落として詰まり感を軽減する。
      // NOTE:
      // - 追加の重い計算は行わず、既に計算している penetrationRatio の最大値だけ保持する。
      // - 近いが接触していない（=wSep だけ高い）ケースでは減速しない。
      float maxPenetrationRatio = 0.0f;

      // ---- 近傍(leaf内) ----
      for (size_t i = 0; i < neighborSlots; ++i) {
        if (!activeNeighbors.test(i)) {
          continue;
        }

        if (cohesionMemories[i] <= 0.0f) {
          continue;
        }

        // 近傍寄与を記憶年齢でフェードさせ、期限切れの瞬間の急変を抑える。
        // ※activeNeighborsはバイナリだが、力は連続的に落ちる。
        const float baseTau = globalSpeciesParams[sid].tau;
        const float tauJitter =
            baseTau * (0.85f + 0.30f * hash01(uint32_t(gIdx) * 1664525u + uint32_t(i) * 1013904223u));
        const float memoryAge = cohesionMemories[i];
        const float memoryFade =
            tauJitter > 1e-6f ? glm::clamp(1.0f - (memoryAge / tauJitter), 0.0f, 1.0f) : 0.0f;

        int gNeighbor = indices[i];
        ++aggregatedNeighborCount;
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (cohesionRangeSq > 0.0f && distSq < cohesionRangeSq) {
          ++closeNeighborCount;
        }
        int neighborSid = buf->speciesIds[gNeighbor];
        const SpeciesParams &neighborParams = globalSpeciesParams[neighborSid];

        // Blender 設定の符号そのまま受け取りつつ、実際の長さは絶対値で扱う。
        float neighborHead = std::abs(neighborParams.bodyHeadLength);
        float neighborTail = std::abs(neighborParams.bodyTailLength);
        float selfHead = std::abs(selfParams.bodyHeadLength);
        float selfTail = std::abs(selfParams.bodyTailLength);
        float neighborRadius = std::max(neighborParams.bodyRadius, 0.0f);
        float selfRadius = std::max(selfParams.bodyRadius, 0.0f);
        float neighborSpan = neighborHead + neighborTail;
        float selfSpan = selfHead + selfTail;
        float closeCheckRange =
            0.5f * (selfSpan + neighborSpan) + (selfRadius + neighborRadius);
        closeCheckRange =
            glm::max(closeCheckRange, selfRadius + neighborRadius);
        float closeCheckRangeSq = closeCheckRange * closeCheckRange;

        // 近接反発（軽量版）
        // カプセル最近接は高コストかつ相手への書き込みで競合しやすいので、
        // ここでは距離ベースの反発に簡略化する（自分自身のみに適用）。
        if (distSq <= closeCheckRangeSq + 1e-6f && distSq > 1e-12f) {
          const float dist = glm::sqrt(distSq);
          const float penetration = glm::max(closeCheckRange - dist, 0.0f);
          const float penetrationRatio = penetration / glm::max(closeCheckRange, 1e-5f);
          maxPenetrationRatio = glm::max(maxPenetrationRatio, penetrationRatio);
          // めり込みが深いほど急激に強くする（魚が重なるのを防ぐ定石）。
          // - 浅い接触では安定性を優先し、過剰反発を避ける
          // - 深いめり込みでは強制的に押し戻す（重なりを残さない）
          float response = penetrationRatio * penetrationRatio;
          response *= response; // 4乗
          // めり込みが深いほど追加で強く押し返す。
          // - 浅い接触: ほぼ影響なし（挙動のガタつき回避）
          // - 深いめり込み: 速やかに解消（重なり残りを抑える）
          // 追加ブーストは penetrationRatio^2 ベースにし、分岐なしで安定させる。
          const float overlapBoost = 1.0f + (penetrationRatio * penetrationRatio) * 3.0f;

          // 係数は速度と separation に応じてスケール。極端な発散は後段の加速度クリップで抑える。
          const float impulse = response * overlapBoost * glm::max(selfParams.separation, 0.02f) *
                                (1.0f + selfParams.maxSpeed) * 18.0f;
          buf->accelerations[gIdx] += (diff * (1.0f / dist)) * (-impulse);
        }

        if (distSq <= 1e-4f)
          continue;

        float dist = glm::sqrt(distSq);
        if (dist < propagationRadius) {
          float neighborStress = buf->stresses[gNeighbor];
          if (neighborStress > selfStress) {
            float stressStrength =
                glm::smoothstep(0.25f, 0.75f, neighborStress);
            float distanceFactor =
                glm::clamp(1.0f - dist / propagationRadius, 0.0f, 1.0f);
            if (distanceFactor > 0.0f) {
              float weight = stressStrength * distanceFactor;
              stressGainSum += neighborStress * weight;
              stressWeightSum += weight;
            }
          }
        }

        // predatorThreat の伝搬（stress より反応を少し速く、半径もやや広げる）
        if (dist < threatPropagationRadius) {
          const float neighborThreat =
              glm::clamp(buf->predatorThreats[gNeighbor], 0.0f, 1.0f);
          // 直接の捕食者影響が無い低脅威まで無制限に伝搬すると「関係ない場所が逃げる」状態になりやすい。
          // ある程度の脅威、または逃避方向（predatorInfluences）がある個体からの伝搬のみ許可する。
          const float neighborInfluence2 = glm::length2(buf->predatorInfluences[gNeighbor]);
          const bool hasDirectInfluence = neighborInfluence2 > 1e-4f;
          if ((hasDirectInfluence || neighborThreat > 0.25f) && neighborThreat > selfThreat) {
            const float threatStrength =
                glm::smoothstep(0.20f, 0.80f, neighborThreat);
            const float distanceFactor = glm::clamp(
                1.0f - dist / threatPropagationRadius, 0.0f, 1.0f);
            if (distanceFactor > 0.0f) {
              const float weight = threatStrength * distanceFactor;
              const float transmittedThreat = neighborThreat * threatTransmission;
              threatGainSum += transmittedThreat * weight;
              threatWeightSum += weight;
            }
          }
        }

        float separationRange = globalSpeciesParams[sid].separationRange;
        if (separationRange <= 1e-4f) {
          // separationRange が0近辺の場合は体長・体幅から最低限の距離を構成。
          float bodyDiameter = std::max(selfRadiusAbs * 2.0f, 0.0f);
          float bodyLength = selfHeadAbs + selfTailAbs;
          separationRange = glm::max(bodyDiameter, bodyLength);
        }
        float wSep = 0.0f;
        if (separationRange > 1e-4f) {
          wSep = 1.0f - (dist / separationRange);
        }
        wSep = glm::clamp(wSep, 0.0f, 1.0f);
        wSep *= memoryFade;
        crowdingWeightSum += wSep;
        if (wSep > 0.35f) {
          ++crowdCloseCount;
        }
        sumSep += (diff * wSep) * (-1.0f);

        // 凝集の重み計算：近いほど強い（距離の正規化を反転）
        float t = glm::clamp(dist / globalSpeciesParams[sid].cohesionRange,
                             0.0f, 1.0f);
        float wCoh = 1.0f - t; // 近いほど強い（0=遠い、1=近い）
        wCoh *= memoryFade;
        // stress に応じて凝集強度を増加（再結集フェーズ強化）
        float stressFactor = 1.0f + selfStress * 0.2f;
        wCoh *= stressFactor;

        // 相対ベクトル（diff）を重み付きで加算（世界座標を使わない）
        sumCohDir += diff * wCoh;
        wCohSum += wCoh;

        // alignment も寿命でフェードさせ、近傍入れ替わりの急変を抑える。
        sumAlign += buf->velocities[gNeighbor] * memoryFade;
      }

      // ---- 近傍(ユニット外: SpatialIndex 球検索) ----
      // こちらはメモリ構造を持たないため、フレーム内の力学合算のみ行う。
      for (int gNeighbor : externalNeighbors) {
        if (gNeighbor < 0) {
          continue;
        }
        ++aggregatedNeighborCount;
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (!(distSq > EPS && distSq < viewRangeSq)) {
          continue;
        }
        if (cohesionRangeSq > 0.0f && distSq < cohesionRangeSq) {
          ++closeNeighborCount;
        }

        // 近接反発（外部近傍版）
        // 近傍キャッシュ(leaf内)に入っていない相手でも、重なりだけは確実に排除する。
        const int neighborSid = buf->speciesIds[gNeighbor];
        const SpeciesParams &neighborParams = globalSpeciesParams[neighborSid];
        const float neighborHeadAbs = std::abs(neighborParams.bodyHeadLength);
        const float neighborTailAbs = std::abs(neighborParams.bodyTailLength);
        const float neighborRadiusAbs = std::max(neighborParams.bodyRadius, 0.0f);
        const float neighborSpan = neighborHeadAbs + neighborTailAbs;
        const float selfSpan = selfHeadAbs + selfTailAbs;
        float closeCheckRange =
            0.5f * (selfSpan + neighborSpan) + (selfRadiusAbs + neighborRadiusAbs);
        closeCheckRange = glm::max(closeCheckRange, selfRadiusAbs + neighborRadiusAbs);
        const float closeCheckRangeSq = closeCheckRange * closeCheckRange;
        if (distSq <= closeCheckRangeSq + 1e-6f && distSq > 1e-12f) {
          const float dist = glm::sqrt(distSq);
          const float penetration = glm::max(closeCheckRange - dist, 0.0f);
          const float penetrationRatio = penetration / glm::max(closeCheckRange, 1e-5f);
          maxPenetrationRatio = glm::max(maxPenetrationRatio, penetrationRatio);
          float response = penetrationRatio * penetrationRatio;
          response *= response; // 4乗
          const float overlapBoost = 1.0f + (penetrationRatio * penetrationRatio) * 3.0f;
          const float impulse = response * overlapBoost * glm::max(selfParams.separation, 0.02f) *
                                (1.0f + selfParams.maxSpeed) * 18.0f;
          buf->accelerations[gIdx] += (diff * (1.0f / dist)) * (-impulse);
        }

        float dist = glm::sqrt(distSq);
        if (dist < propagationRadius) {
          float neighborStress = buf->stresses[gNeighbor];
          if (neighborStress > selfStress) {
            float stressStrength =
                glm::smoothstep(0.25f, 0.75f, neighborStress);
            float distanceFactor =
                glm::clamp(1.0f - dist / propagationRadius, 0.0f, 1.0f);
            if (distanceFactor > 0.0f) {
              float weight = stressStrength * distanceFactor;
              stressGainSum += neighborStress * weight;
              stressWeightSum += weight;
            }
          }
        }

        // predatorThreat の伝搬（外部近傍も同様に加味する）
        if (dist < threatPropagationRadius) {
          const float neighborThreat =
              glm::clamp(buf->predatorThreats[gNeighbor], 0.0f, 1.0f);
          const float neighborInfluence2 = glm::length2(buf->predatorInfluences[gNeighbor]);
          const bool hasDirectInfluence = neighborInfluence2 > 1e-4f;
          if ((hasDirectInfluence || neighborThreat > 0.25f) && neighborThreat > selfThreat) {
            const float threatStrength =
                glm::smoothstep(0.20f, 0.80f, neighborThreat);
            const float distanceFactor = glm::clamp(
                1.0f - dist / threatPropagationRadius, 0.0f, 1.0f);
            if (distanceFactor > 0.0f) {
              const float weight = threatStrength * distanceFactor;
              const float transmittedThreat = neighborThreat * threatTransmission;
              threatGainSum += transmittedThreat * weight;
              threatWeightSum += weight;
            }
          }
        }

        float separationRange = globalSpeciesParams[sid].separationRange;
        if (separationRange <= 1e-4f) {
          float bodyDiameter = std::max(selfRadiusAbs * 2.0f, 0.0f);
          float bodyLength = selfHeadAbs + selfTailAbs;
          separationRange = glm::max(bodyDiameter, bodyLength);
        }
        float wSep = 0.0f;
        if (separationRange > 1e-4f) {
          wSep = 1.0f - (dist / separationRange);
        }
        wSep = glm::clamp(wSep, 0.0f, 1.0f);
        crowdingWeightSum += wSep;
        if (wSep > 0.35f) {
          ++crowdCloseCount;
        }
        sumSep += (diff * wSep) * (-1.0f);

        float t = glm::clamp(dist / globalSpeciesParams[sid].cohesionRange,
                             0.0f, 1.0f);
        float wCoh = 1.0f - t;
        float stressFactor = 1.0f + selfStress * 0.2f;
        wCoh *= stressFactor;

        sumCohDir += diff * wCoh;
        wCohSum += wCoh;
        sumAlign += buf->velocities[gNeighbor];
      }

      // cohesionRange 内の近傍数で φ を更新し、薄さ検出を即時化する。
      if (phiDenom > 0) {
        float phiNow = float(closeNeighborCount) / float(phiDenom);
        phi = glm::clamp(phiNow, 0.0f, 1.0f);
      } else {
        phi = 1.0f;
      }

      // -------------------------------------------------------
      // 葉ユニット重心による局所的な結合補助
      // -------------------------------------------------------
      // 近傍メモリの制限で neighborCount が少なくても、同じ葉に属する
      // 個体の中心方向を弱く加算し、原点ではなく局所クラスタへ戻す。
      const int leafPopulation = static_cast<int>(indices.size());
      if (leafPopulation > 1) {
        float scatterFactor = glm::clamp(1.0f - phi, 0.0f, 1.0f);
        if (scatterFactor > 1e-4f) {
          glm::vec3 sumAll = center * static_cast<float>(leafPopulation);
          glm::vec3 avgOthers =
              (sumAll - pos) / static_cast<float>(leafPopulation - 1);
          glm::vec3 toLeafCenter = avgOthers - pos;
          float centerVecLen2 = glm::length2(toLeafCenter);
          if (centerVecLen2 > EPS) {
            float normalizedDist = 0.0f;
            if (radius > 1e-4f) {
              normalizedDist =
                  glm::clamp(glm::sqrt(centerVecLen2) / glm::max(radius, 1e-3f),
                             0.0f, 1.5f);
            }
            float centerWeight =
                scatterFactor * glm::mix(0.08f, 0.35f, normalizedDist);
            sumCohDir += toLeafCenter * centerWeight;
            wCohSum += centerWeight;
          }
        }
      }

      // 近傍数は leaf 内 + 外部を合算したもので正規化する。
      float invN = 1.0f / float(std::max(aggregatedNeighborCount, 1));
      if (stressWeightSum > 0.0f) {
        float propagatedStress = stressGainSum / stressWeightSum;
        float delta = propagatedStress - selfStress;
        if (delta > 0.0f) {
          float blend = glm::clamp(propagationBlend * dt, 0.0f, 0.6f);
          float updatedStress =
              glm::clamp(selfStress + delta * blend, 0.0f, 1.0f);
          buf->stresses[gIdx] = updatedStress;
          selfStress = updatedStress;
        }
      }

      // predatorThreats は「最大値」ベースで維持しつつ、近傍平均との差分だけを滑らかに追従する。
      if (threatWeightSum > 0.0f) {
        const float propagatedThreat = threatGainSum / threatWeightSum;
        const float delta = propagatedThreat - selfThreat;
        if (delta > 0.0f) {
          const float blend = glm::clamp(threatPropagationBlend * dt, 0.0f, 0.8f);
          const float updatedThreat =
              glm::clamp(selfThreat + delta * blend, 0.0f, 1.0f);
          buf->predatorThreats[gIdx] = std::max(buf->predatorThreats[gIdx], updatedThreat);
          selfThreat = updatedThreat;
        }
      }
      // 分離の最終ベクトル
      glm::vec3 totalSeparation = glm::vec3(0.0f);
      float sepLen2 = glm::length2(sumSep);
      if (sepLen2 > EPS) {
        totalSeparation =
            (sumSep * (1.0f / glm::sqrt(sepLen2))) *
          globalSpeciesParams[sid].separation;
      }

        // -------------------------------------------------------
        // Autonomous control of attraction（混雑時に凝集を弱める）
        // -------------------------------------------------------
        // 近傍が十分に充足している（= 密）ほど、凝集=attraction を自動的に弱める。
        // 目的:
        // - クラスタ内部での「詰まり」を抑え、過剰な中心吸い込みを減らす
        // - ローカル相互作用だけでの回転（ミリング）や分裂/合流を起こしやすくする
        // 注意:
        // - 脅威(threat)が高い局面では群れ維持を優先し、抑制を緩める
        // - 薄い領域(phiが低い)では抑制がほぼ掛からない（外縁は集まりやすい）
          // 近傍数(phi)だけだと maxNeighbors が小さい/飽和している場合に
          // 内側と外縁の差が出ない。そこで wSep 平均（近さ）も混雑度に含める。
          const float crowdPhi = glm::clamp(phi, 0.0f, 1.0f);
          const float crowdPhiSignal = glm::smoothstep(0.75f, 0.98f, crowdPhi);
          const float crowdingAvg =
            aggregatedNeighborCount > 0
              ? glm::clamp(crowdingWeightSum / float(aggregatedNeighborCount), 0.0f, 1.0f)
              : 0.0f;
          const float crowdPackingSignal = glm::smoothstep(0.20f, 0.55f, crowdingAvg);
            const float crowdCloseRatio =
              aggregatedNeighborCount > 0
                ? glm::clamp(float(crowdCloseCount) / float(aggregatedNeighborCount), 0.0f, 1.0f)
                : 0.0f;
            // 近い個体が一定割合を超えたら「密集」とみなす。
            const float crowdCloseSignal = glm::smoothstep(0.10f, 0.35f, crowdCloseRatio);
            const float crowded = glm::max(crowdPhiSignal, glm::max(crowdPackingSignal, crowdCloseSignal));
        constexpr float kAttractionMinScale = 0.35f;
        float autonomousAttractionScale =
          glm::mix(1.0f, kAttractionMinScale, crowded);
        autonomousAttractionScale =
          glm::mix(autonomousAttractionScale, 1.0f, threatLevel);

      // 凝集の最終ベクトル（重みの総和で正規化、原点依存なし）
      glm::vec3 totalCohesion = glm::vec3(0.0f);
      if (wCohSum > EPS) {
        glm::vec3 cohDir = sumCohDir / wCohSum; // 重み付き平均方向
        float cohLen2 = glm::length2(cohDir);
        if (cohLen2 > EPS) {
          localCenterDir = cohDir;
          hasLocalCenterDir = true;
          const float edgeFactor =
              1.0f + glm::clamp(1.0f - phi, 0.0f, 1.0f); // 外縁ほど強化
          totalCohesion = (cohDir * (1.0f / glm::sqrt(cohLen2))) *
                          (globalSpeciesParams[sid].cohesion * edgeFactor);
        }
      }
      // 混雑時は attraction を弱め、クラスタ内部の過凝集を抑える。
      totalCohesion *= autonomousAttractionScale;

      // 整列の最終ベクトル
      glm::vec3 avgAlignVel = sumAlign * invN;
      glm::vec3 totalAlignment = glm::vec3(0.0f);
      glm::vec3 aliDir = avgAlignVel - vel;
      float aliLen2 = glm::length2(aliDir);
      if (aliLen2 > EPS) {
        totalAlignment =
            (aliDir * (1.0f / glm::sqrt(aliLen2))) *
          globalSpeciesParams[sid].alignment;
      }

      if (hasSchoolCenterDir) {
        if (globalSpeciesParams[sid].isPredator) {
          // 捕食者は大クラスタ引力に左右されない。修正ループを防ぐため即スキップ。
          continue;
        }
        float clusterLen2 = glm::length2(schoolCenterDir);
        if (clusterLen2 > EPS) {
          const float clusterDist = glm::sqrt(clusterLen2);
          glm::vec3 globalDir =
              schoolCenterDir * (1.0f / glm::max(clusterDist, 1e-4f));
          // 大クラスタ引力: 基本は中心へ向かう（3D）
          // ただし水平トルクが強い種は「水平に戻ろうとする」ので、
          // 引力も水平成分をやや優先して不自然な上下の引っ張りを弱める。
          glm::vec3 globalDirHoriz = globalDir;
          globalDirHoriz.y = 0.0f;
          const float horizLen2 = glm::length2(globalDirHoriz);
          if (horizLen2 > EPS) {
            globalDirHoriz *= 1.0f / glm::sqrt(horizLen2);

            const float horizontalTorque =
                glm::clamp(globalSpeciesParams[sid].horizontalTorque, 0.0f, 0.1f);
            // horizontalTorque は典型的に 0.005〜0.03 程度なので、0..1 に正規化してブレンド。
            const float horizBias = glm::clamp(horizontalTorque * 30.0f, 0.0f, 1.0f);
            globalDir = glm::normalize(glm::mix(globalDir, globalDirHoriz, horizBias));
          }

          const float dirLen2 = glm::length2(globalDir);
          if (dirLen2 > EPS) {
            globalDir *= 1.0f / glm::sqrt(dirLen2);
            // 大クラスタ引力は半径に依存させず、常に中心方向へ寄せる。
            // - 半径が大きいケースでも誘導が弱まりすぎないようにする。
            // - 中心付近の微小な揺れは、近接時だけ軽く減衰させる。
            const float nearAttenuation =
                glm::smoothstep(0.15f, 2.0f, clusterDist);
            const float clusterPull = baseCohesionStrength * nearAttenuation *
                                      glm::clamp(
                                          schoolWeight *
                                              gSimulationTuning
                                                  .schoolPullCoefficient,
                                          0.0f, 0.9f);
            longTermCohesion += globalDir * clusterPull;
          }
        }
      }

      // 種単位の移動参照球(envelope)から外れた場合の復帰力は、
      // 「同一speciesで複数の独立した群れ」を作ると、全群れが“種の重心”へ
      // 同時に引かれて一斉に向きが変わる副作用が出やすい。
      // ユーザー要望: 群れが急に全体で向きを変える挙動を消す。
      // そのため envelope 復帰力は無効化する（ローカル相互作用のみで復帰を担う）。


      // 旧・長期誘導機能は廃止したため、長期項は school cluster 由来の凝集のみを加算する。
      // alignment は近傍速度の整列（短期）だけで決める。
      const glm::vec3 combinedCohesion = totalCohesion + longTermCohesion;
      const glm::vec3 combinedAlignment = totalAlignment;

      // --- 回転トルクによる向き補正（alignment方向へ向ける） ---
      float velLen2_2 = glm::length2(vel);
      if (velLen2_2 > EPS) {
        // forward2 の正規化と speed を同時に得て、後段での sqrt を増やさない。
        const float velSpeed = glm::sqrt(velLen2_2);
        glm::vec3 forward2 = vel * (1.0f / velSpeed);

        // 合成後のアライメントベクトルがゼロでないかチェック
        float aliLen2_check = glm::length2(combinedAlignment);
        if (aliLen2_check > EPS) {
          glm::vec3 tgt2 =
              combinedAlignment * (1.0f / glm::sqrt(aliLen2_check));
          float dot2 = glm::clamp(glm::dot(forward2, tgt2), -1.0f, 1.0f);

          // `acosf(dot)` は高コストなため、angle = atan2(|cross|, dot) で置き換える。
          // |cross| は軸の正規化で必要なので、sqrt は1回で済む。
          glm::vec3 axis2 = glm::cross(forward2, tgt2);
          const float axisLen2 = glm::length2(axis2);
          if (axisLen2 > EPS) {
            const float axisLen = glm::sqrt(axisLen2);
            const float ang2 = atan2f(axisLen, dot2);
            if (ang2 > 1e-4f) {
              axis2 *= (1.0f / axisLen);
              float rot2 =
                  std::min(ang2, globalSpeciesParams[sid].torqueStrength * dt);
              // maxTurnAngle は「最大曲率（距離あたりの回転量）」なので、
              // 1ステップ上限角は curvature * speed * dt で算出する。
              rot2 = std::min(rot2, globalSpeciesParams[sid].maxTurnAngle * velSpeed * dt);
              glm::vec3 newDir2 = approxRotate(forward2, axis2, rot2);

              // 速度ベクトルを回転後の方向に更新（sqrtの重複を避ける）
              vel = newDir2 * velSpeed;

              // 加速度にもトルク分を加算
              buf->accelerations[gIdx] +=
                  axis2 * ang2 * globalSpeciesParams[sid].torqueStrength;
            }
          }
        }
      }
      // --- 最終的な加速度をバッファに書き込み ---
      if (maxPenetrationRatio > 0.0f) {
        // 密集して動けないときは、その分「速度を落として押し合い」を解消しやすくする。
        // - 速度が高いほど詰まりが悪化しやすいので、速度に比例した抵抗（drag）を与える。
        // - 深いめり込みほど強く、浅い接触ではほぼ効かない。
        const float slowSignal = glm::smoothstep(0.10f, 0.55f, maxPenetrationRatio);
        const float kCrowdingDrag = 10.0f;
        buf->accelerations[gIdx] += (-vel) * (slowSignal * kCrowdingDrag);
      }
      buf->accelerations[gIdx] +=
          totalSeparation + combinedAlignment + combinedCohesion;
    }
  }

  // 大型クラスタ計算後に thread_local バッファの肥大化を抑制
  if (candidates.capacity() > kCandidateCacheLimit) {
    candidates.clear();
    std::vector<std::pair<float, int>>().swap(candidates);
    candidates.reserve(kCandidateCacheLimit);
  }
  if (predatorTargetCandidates.capacity() > kPredatorCacheLimit) {
    predatorTargetCandidates.clear();
    std::vector<int>().swap(predatorTargetCandidates);
    predatorTargetCandidates.reserve(kPredatorCacheLimit);
  }
}

/**
 * 分割が必要か判定する。
 *
 * 判定条件:
 * - Boid数が上限を超過
 * - 半径が閾値を超過
 * - 方向のバラつきが閾値を超過
 */
bool BoidUnit::needsSplit(float splitRadius, float directionVarThresh,
                          int maxBoids) const {
  if (static_cast<int>(indices.size()) > maxBoids)
    return true;
  if (radius > splitRadius)
    return true;

  // 方向のバラつき判定
  if (indices.size() > 1) {
    glm::vec3 avg = glm::vec3(0.00001f);
    for (int gIdx : indices) {
      // 速度ゼロの normalize を避ける（NaN混入防止）。
      avg += safeNormalizeOr(buf->velocities[gIdx], glm::vec3(0.0f, 0.0f, 1.0f));
    }
    avg /= static_cast<float>(indices.size());

    float var = 0.0f;
    for (int gIdx : indices) {
      const glm::vec3 dir =
          safeNormalizeOr(buf->velocities[gIdx], glm::vec3(0.0f, 0.0f, 1.0f));
      var += glm::length(dir - avg);
    }
    var /= static_cast<float>(indices.size());

    if (var > directionVarThresh)
      return true;
  }
  return false;
}

/**
 * 最大分散軸で指定数に分割する。
 *
 * 処理内容:
 * - 最大分散軸を特定
 * - 等間隔でグループ分け
 * - 各グループに新しいBoidUnitを作成
 */
std::vector<BoidUnit *> BoidUnit::split(int numSplits) {
  if (numSplits < 2)
    numSplits = 2;
  if (static_cast<int>(indices.size()) < numSplits)
    numSplits = static_cast<int>(indices.size());

  // 最大分散軸を求める
  int axis = 0;
  float maxVar = 0.0f;
  for (int ax = 0; ax < 3; ++ax) {
    float mean = 0.0f, var = 0.0f;
    for (int gIdx : indices)
      mean += (ax == 0   ? buf->positions[gIdx].x
               : ax == 1 ? buf->positions[gIdx].y
                         : buf->positions[gIdx].z);
    mean /= static_cast<float>(indices.size());

    for (int gIdx : indices) {
      float v = (ax == 0   ? buf->positions[gIdx].x
                 : ax == 1 ? buf->positions[gIdx].y
                           : buf->positions[gIdx].z) -
                mean;
      var += v * v;
    }
    if (var > maxVar) {
      maxVar = var;
      axis = ax;
    }
  }

  // min / max を取り等間隔で分ける
  float minVal = std::numeric_limits<float>::max();
  float maxVal = -std::numeric_limits<float>::max();
  for (int gIdx : indices) {
    float v = (axis == 0   ? buf->positions[gIdx].x
               : axis == 1 ? buf->positions[gIdx].y
                           : buf->positions[gIdx].z);
    minVal = std::min(minVal, v);
    maxVal = std::max(maxVal, v);
  }

  float interval = (maxVal - minVal) / numSplits;
  std::vector<std::vector<int>> groups(numSplits);
  for (int gIdx : indices) {
    float v = (axis == 0   ? buf->positions[gIdx].x
               : axis == 1 ? buf->positions[gIdx].y
                           : buf->positions[gIdx].z);
    int idx = std::min(numSplits - 1, int((v - minVal) / interval));
    groups[idx].push_back(gIdx);
  }

  // 子 BoidUnit を生成
  std::vector<BoidUnit *> result;
  for (const auto &g : groups) {
    if (g.empty())
      continue;
    BoidUnit *u = new BoidUnit();
    u->buf = buf;   // 中央バッファ共有
    u->indices = g; // インデックスだけ保持
    u->computeBoundingSphere();
    result.push_back(u);
  }
  return result;
}

/**
 * 現在のユニットを分割し子ノードとして配置する。
 *
 * 処理内容:
 * - 分割必要性を判定
 * - クラスタリングで4分割
 * - 自身は中間ノードに変更
 */
void BoidUnit::splitInPlace(int maxBoids) {
  if (!needsSplit(80.0f, 0.5f, maxBoids))
    return;

  auto splits = splitByClustering(4);

  indices.clear();
  children = std::move(splits);

  computeBoundingSphere();
}

/**
 * k-means風クラスタリングでindicesをグループ化する。
 *
 * 処理内容:
 * - 初期中心を先頭から選択
 * - 5回の反復でクラスタリング実行
 * - 各グループに新しいBoidUnitを作成
 */
std::vector<BoidUnit *> BoidUnit::splitByClustering(int numClusters) {
  if ((int)indices.size() < numClusters)
    numClusters = static_cast<int>(indices.size());
  if (numClusters < 2)
    numClusters = 2;

  // 初期中心をランダム（ここでは先頭から）に選択
  std::vector<glm::vec3> centers;
  for (int k = 0; k < numClusters; ++k)
    centers.push_back(buf->positions[indices[k]]);

  std::vector<int> assign(indices.size(), 0);

  // 反復回数は少なめに固定
  for (int iter = 0; iter < 5; ++iter) {
    // 割り当て
    for (size_t i = 0; i < indices.size(); ++i) {
      int gI = indices[i];
      float best = std::numeric_limits<float>::max();
      int bestK = 0;
      for (int k = 0; k < numClusters; ++k) {
        float d = glm::distance(buf->positions[gI], centers[k]);
        if (d < best) {
          best = d;
          bestK = k;
        }
      }
      assign[i] = bestK;
    }

    // 中心を再計算
    std::vector<glm::vec3> newCenters(numClusters, glm::vec3(0.00001f));
    std::vector<int> counts(numClusters, 0);
    for (size_t i = 0; i < indices.size(); ++i) {
      int gI = indices[i];
      newCenters[assign[i]] += buf->positions[gI];
      counts[assign[i]]++;
    }
    for (int k = 0; k < numClusters; ++k) {
      if (counts[k] > 0)
        newCenters[k] /= static_cast<float>(counts[k]);
      else
        newCenters[k] = centers[k];
    }
    centers.swap(newCenters);
  }

  // グループごとに global index をまとめる
  std::vector<std::vector<int>> groups(numClusters);
  for (size_t i = 0; i < indices.size(); ++i)
    groups[assign[i]].push_back(indices[i]);

  // 各グループから BoidUnit を生成
  std::vector<BoidUnit *> result;
  for (const auto &g : groups) {
    if (g.empty())
      continue;
    auto *u = new BoidUnit();
    u->buf = buf;             // 中央バッファを共有
    u->indices = g;           // インデックスだけ保持
    u->speciesId = speciesId; // 親ノードの speciesId を継承

    // buf->speciesIds に反映
    for (int gIdx : g) {
      buf->speciesIds[gIdx] = speciesId;
    }

    u->computeBoundingSphere();
    result.push_back(u);
  }
  return result;
}

/**
 * 指定されたユニットと結合可能か判定する。
 *
 * 判定条件:
 * - 中心間距離が閾値以下
 * - 平均速度差が閾値以下
 * - 結合後のBoid数が上限以下
 * - 結合後の半径が上限以下
 */
bool BoidUnit::canMergeWith(const BoidUnit &other, float mergeDist,
                            float velThresh, float maxRadius,
                            int maxBoids) const {
  // 異なる種族同士の merge は禁止する。
  // - mergeWith() は buf->speciesIds を上書きするため、種族の意味（捕食者/獲物等）が崩れる。
  if (speciesId != other.speciesId) {
    return false;
  }

  // 中心間距離
  // `distance()` を使うと sqrt が入るため、distance2 で比較する。
  if (glm::distance2(center, other.center) > (mergeDist * mergeDist))
    return false;

  // 平均速度差
  // `length()` を使うと sqrt が入るため、length2 で比較する。
  if (glm::length2(averageVelocity - other.averageVelocity) >
      (velThresh * velThresh))
    return false;

  // Boid 数上限
  if (static_cast<int>(indices.size() + other.indices.size()) > maxBoids)
    return false;

  // 結合後の中心
  glm::vec3 newCenter =
      (center * static_cast<float>(indices.size()) +
       other.center * static_cast<float>(other.indices.size())) /
      static_cast<float>(indices.size() + other.indices.size());

  // 結合後の半径
  // `distance()` の sqrt をループ内で繰り返さないように maxDist2 を集約する。
  float maxDist2 = 0.0f;
  for (int gIdx : indices) {
    maxDist2 =
        std::max(maxDist2, glm::distance2(newCenter, buf->positions[gIdx]));
  }
  for (int gIdx : other.indices) {
    maxDist2 =
        std::max(maxDist2, glm::distance2(newCenter, buf->positions[gIdx]));
  }
  const float newRadius = (maxDist2 > 0.0f) ? glm::sqrt(maxDist2) : 0.0f;
  return newRadius <= maxRadius;
}

/**
 * 他ユニットを結合する（値渡し版）。
 *
 * 処理内容:
 * - indicesを結合
 * - speciesIdを更新
 * - バウンディングスフィアを再計算
 */
void BoidUnit::mergeWith(const BoidUnit &other) {
  // indices を結合
  indices.insert(indices.end(), other.indices.begin(), other.indices.end());

  // buf->speciesIds を更新
  for (int gIdx : other.indices) {
    buf->speciesIds[gIdx] = speciesId;
  }

  computeBoundingSphere();
}

/**
 * 他ユニットを結合する（ポインタ版）。
 *
 * 処理内容:
 * - indicesを結合
 * - speciesIdを更新
 * - 自ノードを葉に変更
 * - 呼び出し側での木構造更新を前提
 */
void BoidUnit::mergeWith(BoidUnit *other) {
  // indices を結合
  indices.insert(indices.end(), other->indices.begin(), other->indices.end());

  // buf->speciesIds を更新
  for (int gIdx : other->indices) {
    buf->speciesIds[gIdx] = speciesId; // 自ノードの speciesId を適用
  }

  // 自ノードを葉に戻す
  children.clear();

  // バウンディングスフィアを再計算
  computeBoundingSphere();
}

// 兄弟ノード配下の全 Boid に反発を適用
void BoidUnit::addRepulsionToAllBoids(BoidUnit *unit,
                                      const glm::vec3 &repulsion) {
  // NOTE:
  // ここが再帰のままだと、ツリー破損や循環参照が起きた際に
  // ブラウザ側で RangeError("Maximum call stack size exceeded") として
  // 表面化する可能性がある。
  // 反復DFS＋安全弁で「落ちない」方向に寄せる（通常ケースの性能劣化はほぼ無い）。
  if (!unit) {
    return;
  }

  std::vector<std::pair<BoidUnit *, BoidUnit *>> stack;
  stack.reserve(256);
  stack.emplace_back(unit, nullptr);

  constexpr std::size_t kMaxTraversalSteps = 5'000'000;
  std::size_t steps = 0;

  while (!stack.empty()) {
    const auto [current, parent] = stack.back();
    stack.pop_back();
    if (!current) {
      continue;
    }
    if (++steps > kMaxTraversalSteps) {
      // ツリー破損などの異常ケース。無限ループ/クラッシュを避ける。
      return;
    }

    if (current->isBoidUnit()) {
      // ループ内の除算コストを避けるため、半径の逆数を先に計算しておく。
      const float invRadius = 1.0f / (current->radius + 1e-5f);
      for (int gIdx : current->indices) {
        const float d = glm::length(current->buf->positions[gIdx] - current->center);
        // 端ほど 1.0、中心 0.5
        const float w = 0.5f + 0.5f * (d * invRadius);
        current->buf->accelerations[gIdx] += repulsion * w;
      }
      continue;
    }

    for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
      BoidUnit *child = *it;
      if (!child) {
        continue;
      }
      if (child == current || child == parent) {
        continue;
      }
      stack.emplace_back(child, current);
    }
  }
}

int BoidUnit::getMaxID() const {
  // NOTE:
  // デバッグ用途でもツリー破損時に落ちないよう、再帰ではなく反復DFSで最大IDを探索する。
  int maxID = -1; // 初期値を -1 に設定（ID が負の値になることはないと仮定）

  std::vector<std::pair<const BoidUnit *, const BoidUnit *>> stack;
  stack.reserve(256);
  stack.emplace_back(this, nullptr);

  constexpr std::size_t kMaxTraversalSteps = 5'000'000;
  std::size_t steps = 0;

  while (!stack.empty()) {
    const auto [current, parent] = stack.back();
    stack.pop_back();
    if (!current) {
      continue;
    }
    if (++steps > kMaxTraversalSteps) {
      return maxID;
    }

    for (int idx : current->indices) {
      maxID = std::max(maxID, current->buf->ids[idx]);
    }

    for (auto it = current->children.rbegin(); it != current->children.rend(); ++it) {
      const BoidUnit *child = *it;
      if (!child) {
        continue;
      }
      if (child == current || child == parent) {
        continue;
      }
      stack.emplace_back(child, current);
    }
  }

  return maxID;
}