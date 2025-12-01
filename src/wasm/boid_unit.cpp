#include "boid_unit.h"

#include "boids_buffers.h"
#include "boids_tree.h"
#include "pool_accessor.h"
#include "simulation_tuning.h"
#include "spatial_query.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <future>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <stack>
#include <utility>
#include <vector>

namespace {

constexpr float kLeafDensityMinRadius = 0.25f;
constexpr std::size_t kCandidateCacheLimit = 16384; // thread-local reuse上限
constexpr std::size_t kPredatorCacheLimit = 2048;

float computeLeafDensityValue(const BoidUnit *leaf) {
  if (!leaf) {
    return 0.0f;
  }
  float minRadius = kLeafDensityMinRadius;
  if (leaf->speciesId >= 0 &&
      leaf->speciesId < static_cast<int>(globalSpeciesParams.size())) {
    minRadius = glm::max(
        kLeafDensityMinRadius,
        globalSpeciesParams[leaf->speciesId].separationRange * 0.25f);
  }
  const float safeRadius = glm::max(leaf->radius, minRadius);
  const float volume = safeRadius * safeRadius * safeRadius + 1e-3f;
  return static_cast<float>(glm::max<std::size_t>(leaf->indices.size(), 1)) /
         volume;
}

void refreshLeafDensityDirection(BoidUnit *leaf) {
  if (!leaf) {
    return;
  }
  const int sid = leaf->speciesId;
  if (sid < 0 || sid >= static_cast<int>(globalSpeciesParams.size())) {
    leaf->densityDir = glm::vec3(0.00001f);
    leaf->densityDirStrength = 0.0f;
    return;
  }

  const float cohesionRange = globalSpeciesParams[sid].cohesionRange;
  const float localMin = leaf->radius * 2.0f;
  const float localMax = leaf->radius * 4.0f;
  const float baseRange = std::max(std::min(cohesionRange, localMax), localMin);
  const float searchRadius = std::max(baseRange, 0.5f);
  const float searchRadiusSq = searchRadius * searchRadius;

  const float selfDensity = computeLeafDensityValue(leaf);
  glm::vec3 gradient(0.0f);
  float totalWeight = 0.0f;

  BoidTree::instance().forEachLeafIntersectingSphere(
      leaf->center, searchRadius, [&](const SpatialLeaf &leafInfo) {
        const BoidUnit *other = leafInfo.node;
        if (!other || other == leaf) {
          return;
        }
        if (other->speciesId != sid) {
          return;
        }

        glm::vec3 delta = other->center - leaf->center;
        float distSq = glm::length2(delta);
        if (distSq < 1e-6f || distSq > searchRadiusSq) {
          return;
        }

        float dist = glm::sqrt(distSq);
        if (dist <= 1e-6f) {
          return;
        }

        glm::vec3 dir = delta * (1.0f / dist);
        float otherDensity = computeLeafDensityValue(other);
        float densityDelta = glm::max(otherDensity - selfDensity, 0.0f);
        if (densityDelta <= 1e-5f) {
          return;
        }

        float kernel = 1.0f / (1.0f + dist);
        float weight = densityDelta * kernel;
        gradient += dir * weight;
        totalWeight += weight;
      });

  if (totalWeight > 0.0f) {
    float gradLen2 = glm::length2(gradient);
    if (gradLen2 > 1e-6f) {
      glm::vec3 newDir = gradient * (1.0f / glm::sqrt(gradLen2));
      if (leaf->densityDirStrength > 0.0f) {
        glm::vec3 blended = glm::mix(leaf->densityDir, newDir, 0.35f);
        float blendLen2 = glm::length2(blended);
        if (blendLen2 > 1e-6f) {
          blended *= 1.0f / glm::sqrt(blendLen2);
          newDir = blended;
        }
      }
      leaf->densityDir = newDir;
      leaf->densityDirStrength = glm::clamp(totalWeight, 0.05f, 2.0f);
      return;
    }
  }

  leaf->densityDir = glm::vec3(0.0f);
  leaf->densityDirStrength = 0.0f;
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

static void updateLeafKinematics(BoidUnit *unit, float dt) {
  for (size_t i = 0; i < unit->indices.size(); ++i) {
    int gIdx = unit->indices[i];
    int sid = unit->buf->speciesIds[gIdx];
    glm::vec3 velocity = unit->buf->velocities[gIdx];
    glm::vec3 acceleration = unit->buf->accelerations[gIdx];
    glm::vec3 position = unit->buf->positions[gIdx];

    // -----------------------------------------------
    // 捕食者専用の追跡加速度を加算
    // -----------------------------------------------
    if (globalSpeciesParams[sid].isPredator &&
        unit->buf->predatorTargetIndices[gIdx] >= 0) {
      int tgtIdx = unit->buf->predatorTargetIndices[gIdx];
      glm::vec3 tgtPos = unit->buf->positions[tgtIdx];
      glm::vec3 diff = tgtPos - position;
      float d2 = glm::dot(diff, diff);
      if (d2 > 1e-4f) {
        float dist = glm::sqrt(d2);
        glm::vec3 chaseDir = diff / dist;
        float desiredSpeed = globalSpeciesParams[sid].maxSpeed;

        glm::vec3 desiredVel = chaseDir * desiredSpeed;
        glm::vec3 chaseAcc = desiredVel - velocity;
        acceleration += chaseAcc;
      }
    } else if (unit->buf->stresses[gIdx] > 0.1f ||
               glm::length(unit->buf->predatorInfluences[gIdx]) > 0.001f) {
      // -----------------------------------------------
      // 逃避挙動: 捕食者からの影響を逃走加速度に変換
      // -----------------------------------------------
      float currentStress = unit->buf->stresses[gIdx];
      glm::vec3 storedInfluence = unit->buf->predatorInfluences[gIdx];
      float predatorInfluenceMagnitude = glm::length(storedInfluence);

      // threatState は applyPredatorSweep で蓄積した恐怖レベル (0-1)。
      float threatState = unit->buf->predatorThreats[gIdx];
      // 直接の影響が強い場合は threatState を補強（0-1 に正規化して混ぜる）。
      float influenceThreat =
          glm::clamp(predatorInfluenceMagnitude * 0.15f, 0.0f, 1.0f);
      threatState = glm::max(threatState, influenceThreat);
      float threatLevel = glm::clamp(threatState, 0.0f, 1.0f);

      // 逃避方向のブレンド比率を計算（最大 maxEscapeWeight まで）
      float escapeWeight = glm::clamp(
          gSimulationTuning.threatGain * threatLevel, 0.0f,
          gSimulationTuning.maxEscapeWeight);

      // 逃避加速度を計算（捕食者から遠ざかる方向）
      glm::vec3 escapeForce = storedInfluence;
      if (predatorInfluenceMagnitude > 0.001f) {
        glm::vec3 fleeDir = storedInfluence / predatorInfluenceMagnitude;
        float fleeStrength = gSimulationTuning.baseEscapeStrength +
                             gSimulationTuning.escapeStrengthPerThreat *
                                 threatLevel;
        escapeForce = fleeDir * fleeStrength;
      }

      // 通常加速度と逃避加速度を escapeWeight でブレンド
      acceleration = acceleration * (1.0f - escapeWeight) +
                     escapeForce * escapeWeight;

      // threat レベルを時間経過で減衰
      float decayedThreat =
          glm::max(threatState - gSimulationTuning.threatDecay * dt, 0.0f);
      unit->buf->predatorThreats[gIdx] = decayedThreat;
    } else {
      // ストレスも影響もない場合は threat を徐々に減衰
      unit->buf->predatorThreats[gIdx] =
          glm::max(unit->buf->predatorThreats[gIdx] -
                       gSimulationTuning.threatDecay * dt,
                   0.0f);
    }
    float currentStress = unit->buf->stresses[gIdx];
    float stressFactor = 1.0f;

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

    // -----------------------------------------------
    // 共通処理: 速度予測と回転角制限
    // -----------------------------------------------
    glm::vec3 desiredVelocity = velocity + acceleration * dt;
    constexpr float MIN_DIR_LEN2 = 1e-12f;

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

    float dotProduct = glm::dot(oldDir, newDir);
    float angle = acosf(glm::clamp(dotProduct, -1.0f, 1.0f));
    float maxTurnAngle = globalSpeciesParams[sid].maxTurnAngle * stressFactor;

    if (globalSpeciesParams[sid].isPredator &&
        unit->buf->predatorTargetIndices[gIdx] >= 0) {
      maxTurnAngle *= 1.5f; // 捕食者の追跡時は回転制限を緩和
    }
    if (currentStress > 0.7f) {
      float emergencyTurnFactor =
          1.0f + (currentStress - 0.7f) * 5.0f; // 最大6.0倍の旋回能力
      maxTurnAngle *= emergencyTurnFactor;
    }

    if (angle > maxTurnAngle) {
      glm::vec3 axis = glm::cross(oldDir, newDir);
      float axisLength2 = glm::length2(axis);
      if (axisLength2 > 1e-8f) {
        axis /= glm::sqrt(axisLength2);
        float rot = glm::min(angle, maxTurnAngle * dt);
        newDir = approxRotate(oldDir, axis, rot);
      }
    }

    // -----------------------------------------------
    // 共通処理: 水平トルク（tilt補正）
    // -----------------------------------------------
    float tilt = newDir.y;
    if (fabsf(tilt) > 1e-4f) {
      glm::vec3 flatDir = glm::normalize(glm::vec3(newDir.x, 0, newDir.z));
      glm::vec3 axis = glm::cross(newDir, flatDir);
      float flatAngle =
          acosf(glm::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f));
      float axisLength2 = glm::length2(axis);
      if (flatAngle > 1e-4f && axisLength2 > 1e-8f) {
        axis /= glm::sqrt(axisLength2);
        float rot =
            glm::min(flatAngle, globalSpeciesParams[sid].horizontalTorque * dt);
        newDir = approxRotate(newDir, axis, rot);
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
    unit->buf->accelerations[gIdx] = glm::vec3(0.0f);
    unit->buf->predatorInfluences[gIdx] *= 0.5f; // 50%保持で逃走を継続
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
      return;

    // 中心を計算
    center = glm::vec3(0.0f);
    for (int gIdx : indices)
      center += buf->positions[gIdx];
    center /= static_cast<float>(indices.size());

    // 平均距離と分散を計算
    float sum = 0.0f, sum2 = 0.0f;
    for (int gIdx : indices) {
      float d = glm::distance(center, buf->positions[gIdx]);
      sum += d;
      sum2 += d * d;
    }
    float mean = sum / static_cast<float>(indices.size());
    float var = sum2 / static_cast<float>(indices.size()) - mean * mean;
    float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

    // 平均 + α × 標準偏差（α = 1.0）で半径を決定
    radius = mean + 1.0f * stddev;
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
    float sum = 0.0f, sum2 = 0.0f;
    for (size_t i = 0; i < childrenSize; ++i) {
      const BoidUnit *child = childrenData[i];
      float d = glm::distance(center, child->center) + child->radius;
      sum += d;
      sum2 += d * d;
    }
    float mean = sum / static_cast<float>(childrenSize);
    float var = sum2 / static_cast<float>(childrenSize) - mean * mean;
    float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

    radius = mean + 1.0f * stddev;
  }
}
const int targetIndex = 50; // ログを出力する特定の Boid のインデックス

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
  auto applyPredatorSweep = [&](BoidUnit *predatorUnit) -> bool {
    if (!predatorUnit || !predatorUnit->isBoidUnit())
      return false;

    int predatorSid = predatorUnit->speciesId;
    if (!globalSpeciesParams[predatorSid].isPredator)
      return false;

    if (predatorUnit->indices.empty())
      return false;

    // 全非捕食者種の警戒距離から最大値を決定してバウンディング判定で使用
    float maxAlertRadius = 1.0f;
    for (const auto &params : globalSpeciesParams) {
      if (params.isPredator) {
        continue;
      }
      maxAlertRadius =
          std::max(maxAlertRadius, std::max(params.predatorAlertRadius, 0.0f));
    }
    const float predatorEffectRange = maxAlertRadius;

    auto *soa = predatorUnit->buf;
    const glm::vec3 predatorCenter = predatorUnit->center;

    // SpatialIndex を介して捕食者の影響範囲と重なる Boid を直接列挙
    for (int idxA : predatorUnit->indices) {
      spatial_query::forEachBoidInSphere(
          BoidTree::instance(), predatorCenter, predatorEffectRange,
          [&](int idxB, const BoidUnit *leafNode) {
            if (!leafNode || leafNode == predatorUnit) {
              return;
            }
            if (idxA == idxB) {
              return;
            }

            const int sidB = soa->speciesIds[idxB];
            if (globalSpeciesParams[sidB].isPredator) {
              return;
            }

            const SpeciesParams &preyParams = globalSpeciesParams[sidB];
            float alertRadius = std::max(preyParams.predatorAlertRadius, 0.0f);
            if (alertRadius <= 0.0f) {
              alertRadius = maxAlertRadius;
            }
            const float alertRadiusSq = alertRadius * alertRadius;

            const glm::vec3 toTarget =
                soa->positions[idxB] - soa->positions[idxA];
            const float d2 = glm::dot(toTarget, toTarget);
            if (d2 >= alertRadiusSq) {
              return;
            }

            const glm::vec3 escapeDir = glm::normalize(toTarget);
            const float normalizedDistance =
                std::sqrt(std::max(d2, 1e-6f)) / alertRadius;
            float escapeStrength =
                glm::clamp(1.0f - normalizedDistance, 0.0f, 1.0f);
            escapeStrength = escapeStrength * escapeStrength *
                             (3.0f - 2.0f * escapeStrength);

            soa->predatorInfluences[idxB] +=
                escapeDir * escapeStrength * 5.0f;
            soa->predatorThreats[idxB] =
                std::max(soa->predatorThreats[idxB], escapeStrength);

            const float stressLevel =
                glm::clamp(0.4f + escapeStrength * 0.6f, 0.0f, 1.0f);
            soa->stresses[idxB] =
                std::max(soa->stresses[idxB], stressLevel);
          });
    }

    return true;
  };

  bool predatorHandled = applyPredatorSweep(this);
  predatorHandled = applyPredatorSweep(other) || predatorHandled;

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
      for (int idxB : other->indices) {
        // 種族IDチェック: 異なる種族同士では群れ行動を行わない
        int sidB = other->buf->speciesIds[idxB];
        if (sidA != sidB) {
          continue; // 異なる種族とは群れ行動しない
        }

        glm::vec3 diff = buf->positions[idxA] - other->buf->positions[idxB];
        float d2 = glm::dot(diff, diff);

        if (d2 < 100.0f && d2 > 1e-4f) {
          float d = std::sqrt(d2);
          float w = std::max(0.0f, 1.0f - (d / 40.0f));

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
        float cohesionMultiplier = 1.0f;
        float speedMultiplier = 1.0f;

        // 逃走後の再結集フェーズ（中程度ストレス時）
        if (currentStress > 0.2f && currentStress < 0.9f) {
          cohesionMultiplier = 5.0f; // 凝集力を3倍に強化
          speedMultiplier = 1.5f;    // 移動速度を1.5倍に向上
        }
        // 高ストレス時は逃走優先（凝集力抑制）
        else if (currentStress >= 0.9f) {
          cohesionMultiplier = 0.5f; // 凝集力を大幅に抑制
        }

        buf->accelerations[idxA] +=
            (sumVel / float(cnt) - buf->velocities[idxA]) *
            globalSpeciesParams[sidA].alignment;
        buf->accelerations[idxA] +=
          (sumPos / float(cnt) - buf->positions[idxA]) *
          (globalSpeciesParams[sidA].cohesion * cohesionMultiplier *
           longRangeCohesionScale); // 凝集強化
        buf->accelerations[idxA] += sep * globalSpeciesParams[sidA].separation;

        // 再結集中の速度向上を後で適用するため、ストレス情報を保持
        float scaledSpeedBoost = 1.0f + (speedMultiplier - 1.0f) * longRangeCohesionScale;
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

  std::stack<BoidUnit *, std::vector<BoidUnit *>> stack;
  stack.push(this);
  static std::vector<std::future<void>> asyncTasks;
  asyncTasks.clear();
  asyncTasks.reserve(64); // 任意。再確保を抑える
  // スレッドプールで並列タスクをスケジュール（常時有効）
  auto &pool = getThreadPool(); // シングルトン取得
  auto scheduleTask = [&](auto &&task) {
    asyncTasks.emplace_back(pool.enqueue(std::forward<decltype(task)>(task)));
  };
  auto waitScheduledTasks = [&]() {
    for (auto &f : asyncTasks)
      f.get();
    asyncTasks.clear();
    asyncTasks.reserve(64);
  };

  // 第一段階: acceleration をすべて計算
  int firstStageOperations = 0;
  while (!stack.empty() && firstStageOperations < 10000) {
    firstStageOperations++;
    BoidUnit *current = stack.top();
    stack.pop();
    // パフォーマンス最適化: children ベクトルのサイズを事前キャッシュ
    const size_t childrenSize = current->children.size();
    BoidUnit **childrenData = current->children.data(); // 直接ポインタアクセス

    // Leaf は並列実行
    if (current->isBoidUnit()) {
      scheduleTask([current, dt] { current->computeBoidInteraction(dt); });
    } else {
      // パフォーマンス最適化: インデックスアクセスで begin() 呼び出しを削減
      for (size_t i = 0; i < childrenSize; ++i) {
        stack.push(childrenData[i]);
      }

      // 子ユニット間の相互作用をサブツリー単位で非同期処理
      if (childrenSize > 1) {
        scheduleTask([childrenData, childrenSize] {
          // パフォーマンス最適化: 直接ポインタアクセスで operator[]
          // 呼び出しを削減
          for (size_t a = 0; a < childrenSize; ++a) {
            for (size_t b = a + 1; b < childrenSize; ++b) {
              childrenData[a]->applyInterUnitInfluence(childrenData[b]);
            }
          }
        });
      }

      current->computeBoundingSphere();
    }
  }

  waitScheduledTasks();
  // 第二段階: 位置と速度を更新
  stack.push(this);
  int stackOperations = 0; // スタック操作回数をカウント

  while (!stack.empty() && stackOperations < 10000) {
    stackOperations++;
    BoidUnit *current = stack.top();
    stack.pop();

    if (current->isBoidUnit()) {
      BoidUnit *unit = current;
      scheduleTask([unit, dt] { updateLeafKinematics(unit, dt); });
    } else {
      // パフォーマンス最適化: children の直接ポインタアクセス
      const size_t childrenSize = current->children.size();
      BoidUnit **childrenData = current->children.data();
      for (size_t i = 0; i < childrenSize; ++i) {
        stack.push(childrenData[i]);
      }
    }
  }

  waitScheduledTasks();

  // 関数終了時にカウンターをリセット
  callCount--;
}
inline float BoidUnit::easeOut(float t) {
  // イージング関数 (ease-out)
  return t * t * (3.0f - 2.0f * t);
}
inline glm::quat BoidUnit::dirToQuatRollZero(const glm::vec3 &forward) {
  glm::vec3 f = glm::normalize(forward);
  glm::vec3 up(0.0f, 1.0f, 0.0f);
  if (fabsf(glm::dot(f, up)) > 0.99f) { // 平行回避
    up = glm::vec3(1.0f, 0.0f, 0.0f);   // フォールバック
  }
  glm::vec3 right = glm::normalize(glm::cross(up, f));
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
  constexpr int LEAF_DENSITY_STRIDE = 6;
  // Lazy: このフレームで必要になった時だけ更新・キャッシュ確定
  glm::vec3 cachedLeafDensityDir = densityDir;
  float cachedLeafDensityStrength = densityDirStrength;
  bool leafDensityReady = false;

  auto ensureLeafDensity = [&]() {
    if (leafDensityReady) {
      return;
    }

    const bool densityExpired =
        (globalFrame - densityDirFrame) > LEAF_DENSITY_STRIDE * 4;

    if (((globalFrame + id) % LEAF_DENSITY_STRIDE) == 0 || densityExpired) {
      refreshLeafDensityDirection(this);
      densityDirFrame = globalFrame;
    }

    cachedLeafDensityDir = densityDir;
    cachedLeafDensityStrength = densityDirStrength;
    leafDensityReady = true;
  };

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

  // -----------------------------------------------
  // 事前計算しておく定数／準備
  // -----------------------------------------------
  // 非ゼロ判定用イプシロン
  constexpr float EPS =
      1e-8f; // 候補距離を入れてソート/部分ソートするための領域
  // thread_local で確保コストを抑えつつ再利用。上限超過時は後段で縮小する。
  static thread_local std::vector<std::pair<float, int>> candidates;
  static thread_local std::vector<int> predatorTargetCandidates;
  if (candidates.capacity() < indices.size()) {
    candidates.reserve(indices.size());
  }

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
    const float baseCohesionStrength = glm::max(selfParams.cohesion, 0.0f);
    const float returnStrength = glm::max(selfParams.densityReturnStrength, 0.0f);
    const float densityGain = glm::clamp(returnStrength * 0.15f, 0.0f, 8.0f);
    glm::quat selfOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (static_cast<size_t>(gIdx) < buf->orientations.size()) {
      selfOrientation = buf->orientations[gIdx];
    }
    // 自身の魚体をカプセルで近似し、後続の近接チェックに再利用する。
    const CapsuleSegment selfCapsule =
        makeCapsule(pos, selfOrientation, selfParams);
    float threatLevel = glm::clamp(buf->predatorThreats[gIdx], 0.0f, 1.0f);
    const float viewRangeSq = globalSpeciesParams[sid].cohesionRange *
                              globalSpeciesParams[sid].cohesionRange;
    // console.call<void>("log", globalSpeciesParams[sid].species +
    //                              " cohesionRange: " +
    //                              std::to_string(globalSpeciesParams[sid].cohesionRange)
    //                              +
    //                              ", viewRangeSq: " +
    //                              std::to_string(viewRangeSq) +
    //                              ", maxNeighbors: " +
    //                              std::to_string(globalSpeciesParams[sid].maxNeighbors));
    // 視界角度（FOV in degrees）の半分をラジアンに変換
    float halfFovRad =
        glm::radians(globalSpeciesParams[sid].fieldOfViewDeg * 0.5f);
    float cosHalfFov = std::cos(halfFovRad);

    candidates.clear();
    if (globalSpeciesParams[sid].isPredator) {
      // スレッドごとにバッファを共有して動的確保コストを抑える
      int &tgtIdx = buf->predatorTargetIndices[gIdx];
      float &tgtTime = buf->predatorTargetTimers[gIdx];
      // 毎フレームクールダウンを減算（ターゲット消失時も進行）
      tgtTime -= dt;
      // console.call<void>(
      //     "log", "1Predator " + std::to_string(gIdx) +
      //                " checking target index: " + std::to_string(tgtIdx) +
      //                ", time left: " + std::to_string(tgtTime) +
      //                ", dt: " + std::to_string(dt));      //
      //                追跡ターゲットが切れたら新規取得
      bool targetInvalid =
          (tgtIdx < 0) || (tgtTime <= 0.0f) ||
          globalSpeciesParams[buf->speciesIds[tgtIdx]].isPredator;
      if (targetInvalid) {
        tgtIdx = -1;
        predatorTargetCandidates.clear();
        if (predatorTargetCandidates.capacity() < kPredatorCacheLimit) {
          predatorTargetCandidates.reserve(kPredatorCacheLimit);
        }
        const float targetSearchRadius = 100.0f; // 旧来ロジックの探索半径を維持

        // SpatialIndex を通じて周辺の非捕食者を直接収集
        spatial_query::forEachBoidInSphere(
            BoidTree::instance(), pos, targetSearchRadius,
            [&](int candidateIdx, const BoidUnit *leafNode) {
              if (leafNode == this) {
                return;
              }
              if (candidateIdx == gIdx) {
                return;
              }
              const int candidateSpecies = buf->speciesIds[candidateIdx];
              if (globalSpeciesParams[candidateSpecies].isPredator) {
                return;
              }
              predatorTargetCandidates.push_back(candidateIdx);
            });

        if (!predatorTargetCandidates.empty()) {
          int pick =
              rand_range(static_cast<int>(predatorTargetCandidates.size()));
          tgtIdx = predatorTargetCandidates[pick];
          tgtTime = globalSpeciesParams[sid].tau;
        } else {
          // ターゲット候補がいない場合は即再試行できるようリセット
          tgtTime = 0.0f;
        }
      }
      if (tgtIdx >= 0) {
        glm::vec3 diff = buf->positions[tgtIdx] - pos;
        float d2 = glm::dot(diff, diff);
        if (d2 > EPS) {
          // console.call<void>(
          //     "log", "Predator " + std::to_string(gIdx) + " chasing target "
          //     +
          //                std::to_string(tgtIdx) +
          //                ", vel: " +
          //                glm::to_string(buf->velocities[tgtIdx]));
          glm::vec3 direction = glm::normalize(diff);

          // ターゲット方向に一定の加速度を適用
          // buf->accelerations[gIdx] += direction;
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

    // cohesionMemories サイズが indices.size() と一致しない場合があるため確認
    size_t maxMemoryIndex = std::min(indices.size(), cohesionMemories.size());

    for (size_t i = 0; i < maxMemoryIndex; ++i) {
      if (cohesionMemories[i] > 0.0f) {
        cohesionMemories[i] += dt;
        if (cohesionMemories[i] > globalSpeciesParams[sid].tau) {
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
    if (activeCount < globalSpeciesParams[sid].maxNeighbors) {
      // 速度ベクトル vel がほぼゼロかどうかチェック
      float velLen2 = glm::length2(vel);

      bool hasVel = (velLen2 > EPS);
      glm::vec3 forward;
      if (hasVel) {
        float invVelLen = 1.0f / glm::sqrt(velLen2);
        forward = vel * invVelLen;
      }

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
          continue; // 速度ゼロでなければ視界内かどうかを確認（最適化版）
        if (hasVel) {
          // 平方根計算を回避：内積の比較でFOVチェック
          float diffDot = glm::dot(forward, diff);
          float requiredDot = cosHalfFov * glm::sqrt(distSq);
          if (diffDot < requiredDot)
            continue;
        }
        candidates.emplace_back(distSq, (int)i);
      }
    } // -------------------------------------------------------
    // 4. 候補リストから最も近いものを選んで登録
    //    - toAdd = maxNeighbors - activeCount
    //    - 部分ソート (nth_element) で上位toAdd件を取得
    // -------------------------------------------------------
    int toAdd = globalSpeciesParams[sid].maxNeighbors - activeCount;
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
        std::min(indices.size(), cohesionMemories.size());
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

    if (neighborCount == 0) {
      buf->isAttracting[gIdx] = 0;
      buf->attractTimers[gIdx] = 0.0f;

      bool appliedLeafGuidance = false;
      if (densityGain > 1e-4f) {
        ensureLeafDensity();
        if (cachedLeafDensityStrength > 1e-3f) {
          const float dirScale =
              glm::clamp(cachedLeafDensityStrength, 0.25f, 1.5f);
          buf->accelerations[gIdx] += cachedLeafDensityDir *
                                      (baseCohesionStrength * dirScale * densityGain);
          appliedLeafGuidance = true;
        }
      }

      if (!appliedLeafGuidance) {
        if (sid >= 0 && sid < static_cast<int>(buf->speciesCenters.size()) &&
            buf->speciesCounts[sid] > 0) {
          glm::vec3 toCenter = buf->speciesCenters[sid] - pos;
          const float centerLen2 = glm::length2(toCenter);
          if (centerLen2 > EPS) {
            glm::vec3 centerDir = toCenter * (1.0f / glm::sqrt(centerLen2));
            buf->accelerations[gIdx] += centerDir * (baseCohesionStrength * 0.35f);
            appliedLeafGuidance = true;
          }
        }
      }

      continue;
    }

    const int maxNeighbors = globalSpeciesParams[sid].maxNeighbors;
    const float phi =
        maxNeighbors > 0 ? float(neighborCount) / float(maxNeighbors) : 1.0f;

    // densityGain が高いほど薄い領域を早めに検出して引き戻す。
    const float phiThreshold =
      glm::clamp(0.45f + densityGain * 0.04f, 0.5f, 0.85f);
    const float thinLinear = phiThreshold > 1e-5f
                   ? glm::clamp((phiThreshold - phi) / phiThreshold, 0.0f, 1.0f)
                   : 0.0f;
    // 二乗で立ち上がりを強め、薄い方向への移動を明確に抑制する。
    const float thinScale = thinLinear * thinLinear;
    if (densityGain > 1e-4f && thinScale > 0.0f) {
      ensureLeafDensity();
      if (cachedLeafDensityStrength > 1e-3f) {
        const float dirScale =
            glm::clamp(cachedLeafDensityStrength, 0.2f, 1.2f);
        buf->accelerations[gIdx] += cachedLeafDensityDir *
                                    (baseCohesionStrength * thinScale * dirScale * densityGain);
      }
    }

    if (gSimulationTuning.fastAttractStrength > 0.001f) {
      if (phi < 1.0f) {
        // 群れの縁に出た場合は吸引ONでタイマーリセット
        buf->isAttracting[gIdx] = 1;
        buf->attractTimers[gIdx] = globalSpeciesParams[sid].tau;
      } else if (buf->isAttracting[gIdx]) {
        // 内部に戻った場合はタイマーカウントダウン
        buf->attractTimers[gIdx] -= dt;
        if (buf->attractTimers[gIdx] <= 0.0f) {
          buf->isAttracting[gIdx] = 0;
          buf->attractTimers[gIdx] = 0.0f;
        }
      }
    } else {
      buf->isAttracting[gIdx] = 0;
      buf->attractTimers[gIdx] = 0.0f;
    }

    // -------------------------------------------------------
    // Fast Attract 最適化: 近傍ループ結果の再利用
    // -------------------------------------------------------
    // 従来は別途空間クエリを実行していたが、既存の近傍走査で
    // 得られた距離・方向情報を流用することで計算コストを大幅削減。
    // 群れの縁にいるボイドの吸引処理を効率的に実装。
    const bool enableFastAttract =
        buf->isAttracting[gIdx] &&
        gSimulationTuning.fastAttractStrength > 0.001f;
    const float fastAttractSeparation =
        enableFastAttract ? globalSpeciesParams[sid].separationRange : 0.0f;
    const float fastAttractCohesion = enableFastAttract
                                          ? glm::max(globalSpeciesParams[sid].cohesionRange,
                                                     fastAttractSeparation + 0.1f)
                                          : 0.0f;
    const float fastAttractSeparationSq =
        fastAttractSeparation * fastAttractSeparation;
    const float fastAttractCohesionSq =
        fastAttractCohesion * fastAttractCohesion;
    glm::vec3 fastAttractDirSum(0.00001f);  // 吸引方向の累積ベクトル
    int fastAttractDirCount = 0;        // 吸引対象カウント

    if (neighborCount > 0) {
      // 近傍のストレスを集約し、逃走の波を伝搬させる。
    float selfStress = buf->stresses[gIdx];
    float stressGainSum = 0.0f;
    float stressWeightSum = 0.0f;
    const float propagationRadius =
      glm::max(globalSpeciesParams[sid].cohesionRange, 1.0f);
    const float propagationBlend = 0.7f; // dt を掛けて応答速度を調整

      glm::vec3 sumSep = glm::vec3(0.0f);
      glm::vec3 sumAlign = glm::vec3(0.0f);
      glm::vec3 sumCohDir = glm::vec3(0.0f); // 相対ベクトルで凝集を計算
      float wCohSum = 0.0f; // 凝集重みの総和
      for (size_t i = 0; i < neighborSlots; ++i) {
        if (!activeNeighbors.test(i)) {
          continue;
        }

        if (cohesionMemories[i] <= 0.0f) {
          continue;
        }

        int gNeighbor = indices[i];
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (enableFastAttract && distSq > fastAttractSeparationSq &&
            distSq < fastAttractCohesionSq && distSq > EPS) {
          // Fast attract 方向データ収集: 追加クエリ不要の効率実装
          // 分離範囲外かつ凝集範囲内の近傍から吸引方向を算出し、
          // メインの群集力学計算と並行して処理することで性能向上を実現。
          float invDist = glm::inversesqrt(distSq);
          fastAttractDirSum += diff * invDist;
          fastAttractDirCount += 1;
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

        // カプセル同士が重なった場合は強い反発力を両個体に与える。
        if (distSq <= closeCheckRangeSq + 1e-6f) {
          glm::quat neighborOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
          if (static_cast<size_t>(gNeighbor) < buf->orientations.size()) {
            neighborOrientation = buf->orientations[gNeighbor];
          }
          const CapsuleSegment neighborCapsule = makeCapsule(
              buf->positions[gNeighbor], neighborOrientation, neighborParams);
          glm::vec3 closestSelf;
          glm::vec3 closestNeighbor;
          float capsuleDistSq = closestPointsOnSegments(
              selfCapsule.a, selfCapsule.b, neighborCapsule.a,
              neighborCapsule.b, closestSelf, closestNeighbor);
          float clearance = selfCapsule.radius + neighborCapsule.radius;
          float clearanceSq = clearance * clearance;
          if (capsuleDistSq < clearanceSq) {
            glm::vec3 normal = closestSelf - closestNeighbor;
            float normalLenSq = glm::length2(normal);
            if (normalLenSq < 1e-8f) {
              normal = diff;
              normalLenSq = glm::length2(normal);
              if (normalLenSq < 1e-8f) {
                normal = forwardFromOrientation(selfOrientation);
                normalLenSq = glm::length2(normal);
              }
            }
            if (normalLenSq > 1e-8f) {
              float normalLen = glm::sqrt(normalLenSq);
              normal /= normalLen;
              float dist = glm::sqrt(glm::max(capsuleDistSq, 1e-12f));
              float penetration = glm::max(clearance - dist, 0.0f);
              float penetrationRatio = penetration / glm::max(clearance, 1e-5f);
              // めり込み率を二乗した応答係数で狭いほど強く押し返す（10.0fは調整用係数）。
              float response = penetrationRatio * penetrationRatio * 10.0f;
              float selfImpulse = response *
                                  glm::max(selfParams.separation, 0.02f) *
                                  (1.0f + selfParams.maxSpeed);
              float neighborImpulse =
                  response * glm::max(neighborParams.separation, 0.02f) *
                  (1.0f + neighborParams.maxSpeed);

              // 自身を法線方向へ押し出し、相手には反対向きに同じだけ押し返す。
              buf->accelerations[gIdx] += normal * selfImpulse;
              if (static_cast<size_t>(gNeighbor) < buf->accelerations.size()) {
                buf->accelerations[gNeighbor] -= normal * neighborImpulse;
              }
            }
          }
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

        float separationRange = globalSpeciesParams[sid].separationRange;
        if (separationRange <= 1e-4f) {
          // separationRange が0近辺の場合は体長・体幅から最低限の距離を構成。
          float bodyDiameter = std::max(selfRadius * 2.0f, 0.0f);
          float bodyLength = selfHead + selfTail;
          separationRange = glm::max(bodyDiameter, bodyLength);
        }
        float wSep = 0.0f;
        if (separationRange > 1e-4f) {
          wSep = 1.0f - (dist / separationRange);
        }
        wSep = glm::clamp(wSep, 0.0f, 1.0f);
        sumSep += (diff * wSep) * (-1.0f);

        // 凝集の重み計算：近いほど強い（距離の正規化を反転）
        float t = glm::clamp(dist / globalSpeciesParams[sid].cohesionRange, 0.0f, 1.0f);
        float wCoh = 1.0f - t; // 近いほど強い（0=遠い、1=近い）
        // stress に応じて凝集強度を増加（再結集フェーズ強化）
        float stressFactor = 1.0f + selfStress * 0.2f;
        // threat レベルに応じた凝集ブースト（逃避中も群れを保つ）
        float cohesionThreatFactor =
            1.0f + gSimulationTuning.cohesionBoost * threatLevel;
        wCoh *= stressFactor * cohesionThreatFactor;

        // 相対ベクトル（diff）を重み付きで加算（世界座標を使わない）
        sumCohDir += diff * wCoh;
        wCohSum += wCoh;

        sumAlign += buf->velocities[gNeighbor];
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
              normalizedDist = glm::clamp(
                  glm::sqrt(centerVecLen2) / glm::max(radius, 1e-3f), 0.0f,
                  1.5f);
            }
            float centerWeight =
                scatterFactor * glm::mix(0.08f, 0.35f, normalizedDist);
            sumCohDir += toLeafCenter * centerWeight;
            wCohSum += centerWeight;
          }
        }
      }

      float invN = 1.0f / float(neighborCount);
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
      // 分離の最終ベクトル
      glm::vec3 totalSeparation = glm::vec3(0.0f);
      float sepLen2 = glm::length2(sumSep);
      if (sepLen2 > EPS) {
        // threat レベルが高いほど分離を弱める（群れを保つ）
        float separationThreatFactor =
            glm::mix(1.0f, gSimulationTuning.separationMinFactor, threatLevel);
        totalSeparation = (sumSep * (1.0f / glm::sqrt(sepLen2))) *
                          (globalSpeciesParams[sid].separation *
                           separationThreatFactor);
      }

      // 凝集の最終ベクトル（重みの総和で正規化、原点依存なし）
      glm::vec3 totalCohesion = glm::vec3(0.0f);
      if (wCohSum > EPS) {
        glm::vec3 cohDir = sumCohDir / wCohSum; // 重み付き平均方向
        float cohLen2 = glm::length2(cohDir);
        if (cohLen2 > EPS) {
          const float edgeFactor =
              1.0f + glm::clamp(1.0f - phi, 0.0f, 1.0f); // 外縁ほど強化
          totalCohesion = (cohDir * (1.0f / glm::sqrt(cohLen2))) *
                          (globalSpeciesParams[sid].cohesion * edgeFactor);
        }
      }

      // 整列の最終ベクトル
      glm::vec3 avgAlignVel = sumAlign * invN;
      glm::vec3 totalAlignment = glm::vec3(0.0f);
      glm::vec3 aliDir = avgAlignVel - vel;
      float aliLen2 = glm::length2(aliDir);
      if (aliLen2 > EPS) {
        // threat レベルに応じた整列ブースト（逃避時も速度を揃えやすく）
        float alignmentThreatFactor =
            1.0f + gSimulationTuning.alignmentBoost * threatLevel;
        totalAlignment = (aliDir * (1.0f / glm::sqrt(aliLen2))) *
                         (globalSpeciesParams[sid].alignment *
                          alignmentThreatFactor);
      }

      // -------------------------------------------------------
      // 薄い領域からの引き戻しと進行方向の抑制
      // -------------------------------------------------------
      if (densityGain > 1e-4f && thinScale > 0.0f) {
        ensureLeafDensity();
        if (cachedLeafDensityStrength > 1e-3f) {
          const float thinWeight = thinScale * densityGain * (1.0f - threatLevel);
          if (thinWeight > 1e-4f) {
            const float denseDirScale =
                glm::clamp(cachedLeafDensityStrength, 0.3f, 1.5f);
            totalCohesion += cachedLeafDensityDir *
                             (baseCohesionStrength * thinWeight *
                              denseDirScale * 0.6f);
            totalAlignment += cachedLeafDensityDir *
                              (globalSpeciesParams[sid].alignment *
                               thinWeight * 0.35f);

            float velLen2 = glm::length2(vel);
            if (velLen2 > EPS) {
              glm::vec3 forwardDir = vel * (1.0f / glm::sqrt(velLen2));
              float denseDot = glm::dot(forwardDir, cachedLeafDensityDir);
              if (denseDot < 0.0f) {
                float resist = (-denseDot) * thinWeight * denseDirScale;
                buf->accelerations[gIdx] +=
                    cachedLeafDensityDir * (baseCohesionStrength * resist);
              }
            }
          }
        }
      }

      // -------------------------------------------------------
      // Fast Attract 加速度適用: 収集データからの最終計算
      // -------------------------------------------------------
      if (enableFastAttract && fastAttractDirCount > 0) {
        // 近傍走査で収集した方向ベクトルから平均方向を算出し、
        // 群れ復帰のための吸引加速度として適用。
        // 既存データ活用により追加の空間探索コストを回避。
        glm::vec3 avgDir = fastAttractDirSum / float(fastAttractDirCount);
        float avgLen2 = glm::length2(avgDir);
        if (avgLen2 > EPS) {
          avgDir *= 1.0f / glm::sqrt(avgLen2);
          const float attractScale =
              gSimulationTuning.fastAttractStrength *
              globalSpeciesParams[sid].lambda;
          const glm::vec3 desiredVel =
              avgDir * globalSpeciesParams[sid].maxSpeed;
          const glm::vec3 attractAcc = (desiredVel - vel) * attractScale;
          buf->accelerations[gIdx] += attractAcc;
        }
      }

      // --- 回転トルクによる向き補正（alignment方向へ向ける） ---
      float velLen2_2 = glm::length2(vel);
      if (velLen2_2 > EPS) {
        glm::vec3 forward2 = vel * (1.0f / glm::sqrt(velLen2_2));

        // “totalAlignment” がゼロベクトルでないかチェック
        float aliLen2_check = glm::length2(totalAlignment);
        if (aliLen2_check > EPS) {
          glm::vec3 tgt2 = totalAlignment * (1.0f / glm::sqrt(aliLen2_check));
          float dot2 = glm::clamp(glm::dot(forward2, tgt2), -1.0f, 1.0f);
          float ang2 = acosf(dot2);

          if (ang2 > 1e-4f) {
            glm::vec3 axis2 = glm::cross(forward2, tgt2);
            float axisLen2 = glm::length2(axis2);
            if (axisLen2 > EPS) {
              axis2 *= (1.0f / glm::sqrt(axisLen2));
              float rot2 =
                  std::min(ang2, globalSpeciesParams[sid].torqueStrength * dt);
              rot2 = std::min(rot2, globalSpeciesParams[sid].maxTurnAngle);
              glm::vec3 newDir2 = approxRotate(forward2, axis2, rot2);

              // 速度ベクトルを回転後の方向に更新
              vel = newDir2 * glm::length(vel);

              // 加速度にもトルク分を加算
              buf->accelerations[gIdx] +=
                  axis2 * ang2 * globalSpeciesParams[sid].torqueStrength;
            }
          }
        }
      }
      // if (targetIndex == gIdx) {
      //   console.call<void>("log", "UPDATE:L gIdx=" + std::to_string(gIdx) +
      //                                 " dt=" + std::to_string(dt) + "
      //                                 stress=" +
      //                                 std::to_string(buf->stresses[gIdx]));
      // }
      // --- 最終的な加速度をバッファに書き込み ---
      buf->accelerations[gIdx] +=
          totalSeparation + totalAlignment + totalCohesion;

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
    for (int gIdx : indices)
      avg += glm::normalize(buf->velocities[gIdx]);
    avg /= static_cast<float>(indices.size());

    float var = 0.0f;
    for (int gIdx : indices)
      var += glm::length(glm::normalize(buf->velocities[gIdx]) - avg);
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
    u->buf = buf;   // 中央バッファを共有
    u->indices = g; // インデックスだけ保持
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
  // 中心間距離
  if (glm::distance(center, other.center) > mergeDist)
    return false;

  // 平均速度差
  if (glm::length(averageVelocity - other.averageVelocity) > velThresh)
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
  float newRadius = 0.0f;
  for (int gIdx : indices)
    newRadius =
        std::max(newRadius, glm::distance(newCenter, buf->positions[gIdx]));
  for (int gIdx : other.indices)
    newRadius =
        std::max(newRadius, glm::distance(newCenter, buf->positions[gIdx]));

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
  if (unit->isBoidUnit()) {
    for (int gIdx : unit->indices) {
      float d = glm::length(unit->buf->positions[gIdx] - unit->center);
      float w =
          0.5f + 0.5f * (d / (unit->radius + 1e-5f)); // 端ほど 1.0、中心 0.5
      unit->buf->accelerations[gIdx] += repulsion * w;
    }
  } else {
    for (auto *c : unit->children)
      addRepulsionToAllBoids(c, repulsion);
  }
}

int BoidUnit::getMaxID() const {
  int maxID = -1; // 初期値を -1 に設定（ID が負の値になることはないと仮定）
  for (int idx : indices) {
    if (buf->ids[idx] > maxID) {
      maxID = buf->ids[idx];
    }
  }

  // 子ノードがある場合、再帰的に最大 ID を取得
  for (const auto *child : children) {
    if (child) {
      maxID = std::max(maxID, child->getMaxID());
    }
  }

  return maxID;
}