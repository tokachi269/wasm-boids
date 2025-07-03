#include "boid_unit.h"
#include "boids_tree.h"
#include "pool_accessor.h"
#include <algorithm>
#include <emscripten/val.h>
#include <future>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <random>
#include <stack>
#include <string>
#include <vector>

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
      return;    // 子ノードの中心を計算 - パフォーマンス最適化
    center = glm::vec3(0.0f);
    const size_t childrenSize = children.size();
    const BoidUnit* const* childrenData = children.data();
    
    for (size_t i = 0; i < childrenSize; ++i) {
      center += childrenData[i]->center;
    }
    center /= static_cast<float>(childrenSize);

    // 子ノード中心までの平均距離 + 子ノード半径 - パフォーマンス最適化
    float sum = 0.0f, sum2 = 0.0f;
    for (size_t i = 0; i < childrenSize; ++i) {
      const BoidUnit* child = childrenData[i];
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
  if (frameCount % 6 != 0) {
    // 再構築頻度でない場合は何もしない
    return;
  }

  // 捕食者ユニットなら、トップ階層から木構造を使って再帰的に探索する
  if (isBoidUnit() && globalSpeciesParams[speciesId].isPredator) {
    // トップ階層から探索を開始
    BoidUnit *root = this->topParent ? this->topParent : this;

    std::stack<BoidUnit *> stack;
    stack.push(root);

    constexpr float predatorRange = 3.0f;
    constexpr float predatorRangeSq = predatorRange * predatorRange;
    constexpr float predatorEffectRange = 8.0f;

    while (!stack.empty()) {
      BoidUnit *current = stack.top();
      stack.pop();

      // 捕食者は radius を無視して影響範囲で判定
      glm::vec3 diff = current->center - center;
      float dist2 = glm::dot(diff, diff);
      float range = predatorEffectRange + current->radius;

      // ユニットが重なっていない場合はスキップ
      if (dist2 > range * range)
        continue;

      if (current->isBoidUnit()) {
        for (int idxA : indices) {
          for (int idxB : current->indices) {
            int sidB = current->buf->speciesIds[idxB];
            if (globalSpeciesParams[sidB].isPredator)
              continue;

            glm::vec3 toTarget =
                current->buf->positions[idxB] - buf->positions[idxA];
            float d2 = glm::dot(toTarget, toTarget);            if (d2 < predatorRangeSq) {
              glm::vec3 escapeDir = glm::normalize(
                  buf->positions[idxB] - current->buf->positions[idxA]);
              
              // 距離に応じたストレスレベルを計算
              float distance = std::sqrt(d2);
              float normalizedDistance = distance / predatorRange; // 距離を正規化
              float escapeStrength = glm::clamp(1.0f - normalizedDistance, 0.0f, 1.0f);
              escapeStrength = escapeStrength * escapeStrength * (3.0f - 2.0f * escapeStrength); // イージング関数
              
              // 距離に応じたストレスレベル（近いほど高ストレス）
              float stressLevel = 0.3f + escapeStrength * 0.7f; // 0.3-1.0の範囲
              
              // 既存のストレスより高い場合のみ更新（パニック状態の維持）
              if (buf->stresses[idxB] < stressLevel) {
                buf->stresses[idxB] = stressLevel;
              }
              
              buf->predatorInfluences[idxB] += escapeDir * escapeStrength;
            }
          }
        }      } else {
        // 中間ノード: 子ノードを積む - パフォーマンス最適化
        const size_t childrenSize = current->children.size();
        BoidUnit** childrenData = current->children.data();
        for (size_t i = 0; i < childrenSize; ++i) {
          stack.push(childrenData[i]);
        }
      }
    }
  }

  // 通常の影響処理（非捕食者または通常時）
  if (!indices.empty() && !other->indices.empty()) {
    for (int idxA : indices) {
      glm::vec3 sumVel = glm::vec3(0.00001f);
      glm::vec3 sumPos = glm::vec3(0.00001f);
      glm::vec3 sep = glm::vec3(0.00001f);
      int cnt = 0;
      int sidA = speciesId;      for (int idxB : other->indices) {
        // GLMベクトルアクセス最適化: 位置成分を直接計算
        const glm::vec3& posA = buf->positions[idxA];
        const glm::vec3& posB = other->buf->positions[idxB];
        
        float dx = posA.x - posB.x;
        float dy = posA.y - posB.y;
        float dz = posA.z - posB.z;
        float d2 = dx * dx + dy * dy + dz * dz;

        if (d2 < 100.0f && d2 > 1e-4f) {
          float d = std::sqrt(d2);
          float w = std::max(0.0f, 1.0f - (d / 40.0f));

          sumVel += other->buf->velocities[idxB] * w;
          sumPos += posB * w;
          
          // 分離力計算も最適化
          glm::vec3 diff(dx, dy, dz);
          sep += (diff / (d2 + 1.0f)) * w;
          ++cnt;
        }
      }      if (cnt > 0) {
        // ストレス時は群れ行動を抑制（特に凝集力）
        float stressReduction = 1.0f;
        if (buf->stresses[idxA] > 0.3f) {
          // ストレスが高いほど群れ行動を抑制
          stressReduction = 1.0f - (buf->stresses[idxA] - 0.3f) * 0.8f; // 最大80%抑制
          stressReduction = glm::clamp(stressReduction, 0.1f, 1.0f); // 最低10%は維持
        }
        
        buf->accelerations[idxA] +=
            (sumVel / float(cnt) - buf->velocities[idxA]) *
            globalSpeciesParams[sidA].alignment * stressReduction;
        buf->accelerations[idxA] +=
            (sumPos / float(cnt) - buf->positions[idxA]) *
            globalSpeciesParams[sidA].cohesion * stressReduction * 0.5f; // 凝集力は特に抑制
        buf->accelerations[idxA] +=
            sep * (globalSpeciesParams[sidA].separation * 0.5f); // 分離力はストレス時も維持
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
  std::stack<BoidUnit *, std::vector<BoidUnit *>> stack;
  stack.push(this);
  static std::vector<std::future<void>> asyncTasks;
  asyncTasks.reserve(64);       // 任意。再確保を抑える
  auto &pool = getThreadPool(); // シングルトン取得
  // 第一段階: acceleration をすべて計算
  while (!stack.empty()) {
    BoidUnit *current = stack.top();
    stack.pop();
  // パフォーマンス最適化: children ベクトルのサイズを事前キャッシュ
  const size_t childrenSize = current->children.size();
  BoidUnit** childrenData = current->children.data(); // 直接ポインタアクセス
  
  // Leaf は並列実行
  if (current->isBoidUnit()) {
    asyncTasks.emplace_back(
        pool.enqueue([current, dt] { current->computeBoidInteraction(dt); }));
  } else {
    // パフォーマンス最適化: インデックスアクセスで begin() 呼び出しを削減
    for (size_t i = 0; i < childrenSize; ++i) {
      stack.push(childrenData[i]);
    }

    // 子ユニット間の相互作用をサブツリー単位で非同期処理
    if (childrenSize > 1) {
      asyncTasks.emplace_back(pool.enqueue([childrenData, childrenSize] {
        // パフォーマンス最適化: 直接ポインタアクセスで operator[] 呼び出しを削減
        for (size_t a = 0; a < childrenSize; ++a) {
          for (size_t b = a + 1; b < childrenSize; ++b) {
            childrenData[a]->applyInterUnitInfluence(childrenData[b]);
          }
        }
      }));
    }

      current->computeBoundingSphere();
    }
  }

  for (auto &f : asyncTasks)
    f.get();
  asyncTasks.clear();

  // 第二段階: acceleration 適用 → velocity / position 更新
  stack.push(this);
  while (!stack.empty()) {
    BoidUnit *current = stack.top();
    stack.pop();

    if (current->isBoidUnit()) {
      for (size_t i = 0; i < current->indices.size(); ++i) {
        int gIdx = current->indices[i];
        int sid = current->buf->speciesIds[gIdx];
        glm::vec3 velocity = current->buf->velocities[gIdx];
        glm::vec3 acceleration = current->buf->accelerations[gIdx];
        glm::vec3 position = current->buf->positions[gIdx];
        emscripten::val console = emscripten::val::global("console");

        // -----------------------------------------------
        // 捕食者専用の追跡加速度を加算
        // -----------------------------------------------
        if (globalSpeciesParams[sid].isPredator &&
            current->buf->predatorTargetIndices[gIdx] >= 0) {
          int tgtIdx = current->buf->predatorTargetIndices[gIdx];
          glm::vec3 tgtPos = current->buf->positions[tgtIdx];
          glm::vec3 diff = tgtPos - position;
          float d2 = glm::dot(diff, diff);
          // if (targetIndex == gIdx) {
          //   console.call<void>("log",
          //                      "PRE: gIdx=" + std::to_string(gIdx) +
          //                          " vel=" + glm::to_string(velocity) +
          //                          " acc=" + glm::to_string(acceleration) +
          //                          " pos=" + glm::to_string(position));
          // }
          if (d2 > 1e-4f) {
            float dist = glm::sqrt(d2);
            glm::vec3 chaseDir = diff / dist;
            // if (targetIndex == gIdx) {
            //   console.call<void>("log",
            //                      "CHASE: dist=" + std::to_string(dist) +
            //                          " tgtPos=" + glm::to_string(tgtPos) +
            //                          " chaseDir=" +
            //                          glm::to_string(chaseDir));
            // }
            // 到着減速: 近づくほど減速して振動を防ぐ
            float desiredSpeed = globalSpeciesParams[sid].maxSpeed;

            glm::vec3 desiredVel = chaseDir * desiredSpeed;
            glm::vec3 chaseAcc = desiredVel - velocity;
            // if (targetIndex == gIdx) {
            //   console.call<void>(
            //       "log",
            //       "RESULT: desiredVel=" + glm::to_string(desiredVel) +
            //           " chaseAcc=" + glm::to_string(chaseAcc) +
            //           " accMagnitude=" +
            //           std::to_string(glm::length(chaseAcc)));
            // }
            acceleration += chaseAcc;
          }
          // -----------------------------------------------
          // 共通処理: stress/predatorInfluence のブレンド(被捕食者のみ)
          // -----------------------------------------------        } else if (current->buf->stresses[gIdx] > 0.0f) {
          // ストレスレベルに応じた逃走力の重み調整
          float currentStress = current->buf->stresses[gIdx];
          float escapeWeight = 0.3f; // 基本逃走重み
          
          if (currentStress > 0.7f) {
            // パニック状態：逃走力を最優先（群れ行動を大幅に抑制）
            escapeWeight = 0.9f;
          } else if (currentStress > 0.4f) {
            // 中程度のストレス：逃走力を重視
            escapeWeight = 0.7f;
          }
          
          // predatorInfluence の大きさを確認して調整
          float predatorInfluenceMagnitude = glm::length(current->buf->predatorInfluences[gIdx]);
          if (predatorInfluenceMagnitude > 0.01f) {
            // 逃走力が存在する場合のみ混合、そうでなければ通常行動
            acceleration = acceleration * (1.0f - escapeWeight) +
                          current->buf->predatorInfluences[gIdx] * escapeWeight;
          }
          
          // console.call<void>("log", "STRESS: gIdx=" + std::to_string(gIdx) +
          //                               " stress=" + std::to_string(currentStress) +
          //                               " escapeWeight=" + std::to_string(escapeWeight) +
          //                               " acc=" + glm::to_string(acceleration));
        }// ストレス時の緊急回避機能強化
        float currentStress = current->buf->stresses[gIdx];
        float stressFactor = 1.0f + currentStress * 1.2f;
        
        // ストレス時の旋回能力向上（非線形な反応）
        float panicFactor = 1.0f;
        if (currentStress > 0.3f) {
          // ストレスが0.3以上で急激に旋回能力が向上
          float panicLevel = (currentStress - 0.3f) / 0.7f; // 0.3-1.0を0-1にマップ
          panicFactor = 1.0f + panicLevel * panicLevel * 2.5f; // 非線形に増加
        }
        
        float maxSpeed = globalSpeciesParams[sid].maxSpeed * stressFactor;

        // -----------------------------------------------
        // 共通処理: 速度予測と回転角制限
        // -----------------------------------------------
        glm::vec3 desiredVelocity = velocity + acceleration * dt;
        glm::vec3 oldDir = glm::normalize(velocity);
        glm::vec3 newDir = glm::normalize(desiredVelocity);
        float speed = glm::length(desiredVelocity);

        float dotProduct = glm::dot(oldDir, newDir);
        float angle = acosf(glm::clamp(dotProduct, -1.0f, 1.0f));

        // ストレス時の旋回角度大幅向上（パニック状態では3倍以上の旋回能力）
        float maxTurnAngle = globalSpeciesParams[sid].maxTurnAngle * stressFactor * panicFactor;
        
        // 捕食者は回転制限を緩和（必要に応じて）
        if (globalSpeciesParams[sid].isPredator &&
            current->buf->predatorTargetIndices[gIdx] >= 0) {
          maxTurnAngle *= 1.5f; // 捕食者の追跡時は回転制限を緩和
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
            float rot = glm::min(
                flatAngle, globalSpeciesParams[sid].horizontalTorque * dt);
            newDir = approxRotate(newDir, axis, rot);
          }
        }

        // -----------------------------------------------
        // 共通処理: 速度クランプと位置更新
        // -----------------------------------------------
        float finalSpeed =
            glm::clamp(speed, globalSpeciesParams[sid].minSpeed, maxSpeed);        current->buf->velocities[gIdx] = newDir * finalSpeed;
        current->buf->positions[gIdx] += current->buf->velocities[gIdx] * dt;
        current->buf->accelerations[gIdx] = glm::vec3(0.0f);
        current->buf->predatorInfluences[gIdx] = glm::vec3(0.0f); // 次フレーム用にリセット
        current->buf->orientations[gIdx] = BoidUnit::dirToQuatRollZero(newDir);// ストレスの時間経過による減少（改善版）
        if (current->buf->stresses[gIdx] > 0.0f) {
          float currentStress = current->buf->stresses[gIdx];
          
          // ストレスレベルに応じた減少率（高ストレス時はゆっくり回復）
          float baseDecayRate = 0.4f;
          float stressDecayRate = baseDecayRate;
          
          if (currentStress > 0.7f) {
            // パニック状態（ストレス0.7以上）では回復が遅い
            stressDecayRate = baseDecayRate * 0.3f;
          } else if (currentStress > 0.4f) {
            // 中程度のストレスでは通常の回復
            stressDecayRate = baseDecayRate * 0.7f;
          }
          // 軽度のストレス（0.4以下）では通常の回復率
          
          current->buf->stresses[gIdx] -= BoidUnit::easeOut(dt * stressDecayRate);
          if (current->buf->stresses[gIdx] < 0.0f) {
            current->buf->stresses[gIdx] = 0.0f;
          }
        }
      }    } else {
      // パフォーマンス最適化: children の直接ポインタアクセス
      const size_t childrenSize = current->children.size();
      BoidUnit** childrenData = current->children.data();
      for (size_t i = 0; i < childrenSize; ++i) {
        stack.push(childrenData[i]);
      }
    }
  }
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
  glm::vec3 separation;
  glm::vec3 alignment;
  glm::vec3 cohesion;

  int gIdx = 0;
  glm::vec3 pos;
  glm::vec3 vel;
  int sid = -1;
  static std::mt19937 rng{std::random_device{}()};
  // -----------------------------------------------
  // 事前計算しておく定数／準備
  // -----------------------------------------------
  // 非ゼロ判定用イプシロン
  constexpr float EPS = 1e-8f;

  // 候補距離を入れてソート/部分ソートするための領域
  std::vector<std::pair<float, int>> candidates;
  if (candidates.capacity() < indices.size()) {
    candidates.reserve(indices.size());
  }

  // -----------------------------------------------
  // 各 Boid（leafノード内）ごとの反復
  // -----------------------------------------------
  for (size_t index = 0; index < indices.size();
       ++index) { // -------------------------------------------------------
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
    const float viewRangeSq = globalSpeciesParams[sid].cohesionRange *
                              globalSpeciesParams[sid].cohesionRange;
    emscripten::val console = emscripten::val::global("console");
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
      int &tgtIdx = buf->predatorTargetIndices[gIdx];
      float &tgtTime = buf->predatorTargetTimers[gIdx];
      // console.call<void>(
      //     "log", "1Predator " + std::to_string(gIdx) +
      //                " checking target index: " + std::to_string(tgtIdx) +
      //                ", time left: " + std::to_string(tgtTime) +
      //                ", dt: " + std::to_string(dt));      //
      //                追跡ターゲットが切れたら新規取得
      if (tgtTime <= 0.0f || tgtIdx < 0 ||
          globalSpeciesParams[buf->speciesIds[tgtIdx]].isPredator) {
        if (tgtTime <= -globalSpeciesParams[sid].tau) {
          // console.call<void>(
          //     "log", "Predator " + std::to_string(gIdx) +
          //                " checking target index: " + std::to_string(tgtIdx)
          //                +
          //                ", time left: " + std::to_string(tgtTime));          // 親ユニットの子ノードから近いユニットを探索 - パフォーマンス最適化
          std::vector<BoidUnit *> candidateUnits;
          if (parent) {
            const size_t parentChildrenSize = parent->children.size();
            BoidUnit** parentChildrenData = parent->children.data();
            
            // ベクトル成分をキャッシュしてGLMアクセスを最適化
            const float centerX = center.x;
            const float centerY = center.y; 
            const float centerZ = center.z;
            
            for (size_t i = 0; i < parentChildrenSize; ++i) {
              BoidUnit* unit = parentChildrenData[i];
              if (unit == this) continue;
              
              // GLMベクトルアクセス最適化: 成分を直接計算
              const glm::vec3& unitCenter = unit->center;
              float dx = unitCenter.x - centerX;
              float dy = unitCenter.y - centerY;
              float dz = unitCenter.z - centerZ;
              float d2 = dx * dx + dy * dy + dz * dz;
              
              if (d2 < 100.0f) {
                candidateUnits.push_back(unit);
              }
            }
          }

          // 候補ユニット内の Boid をランダムに選択
          if (!candidateUnits.empty()) {
            std::uniform_int_distribution<int> unitDist(
                0, static_cast<int>(candidateUnits.size() - 1));
            BoidUnit *selectedUnit = candidateUnits[unitDist(rng)];

            if (!selectedUnit->indices.empty()) {
              std::uniform_int_distribution<int> boidDist(
                  0, static_cast<int>(selectedUnit->indices.size() - 1));
              tgtIdx = selectedUnit->indices[boidDist(rng)];
              tgtTime = globalSpeciesParams[sid].tau;
            } else {
              tgtIdx = -1;
              tgtTime = 0.0f;
            }
          }
        }
      }
      if (tgtIdx >= 0) {
        tgtTime -= dt * 0.1f;
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
    for (size_t i = 0; i < indices.size(); ++i) {
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
        if (activeNeighbors.test(i) || cohesionMemories[i] > 0.0f)
          continue;

        int gNeighbor = indices[i];
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (distSq >= viewRangeSq)
          continue;

        // 速度ゼロでなければ視界内かどうかを確認
        if (hasVel) {
          float invDist = 1.0f / glm::sqrt(distSq);
          glm::vec3 diffNorm = diff * invDist;
          float dotVal = glm::dot(forward, diffNorm);
          if (dotVal < cosHalfFov)
            continue;
        }

        candidates.emplace_back(distSq, (int)i);
      }
    } // -------------------------------------------------------
    // 4. 候補リストから最も近いものを選んで登録
    //    - toAdd = maxNeighbors - activeCount
    //    - 部分ソート (nth_element) で上位toAdd件を取得
    // -------------------------------------------------------
    int toAdd = globalSpeciesParams[sid].maxNeighbors - activeNeighbors.count();
    if (toAdd > 0 && !candidates.empty()) {
      if ((int)candidates.size() > toAdd) {
        std::nth_element(candidates.begin(), candidates.begin() + toAdd,
                         candidates.end(),
                         [](auto &a, auto &b) { return a.first < b.first; });
        for (int k = 0; k < toAdd; ++k) {
          int idx2 = candidates[k].second;
          cohesionMemories[idx2] = dt;
          activeNeighbors.set(idx2);
        }
      } else {
        for (auto &pr : candidates) {
          int idx2 = pr.second;
          cohesionMemories[idx2] = dt;
          activeNeighbors.set(idx2);
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
    const int neighborCount = static_cast<int>(activeNeighbors.count());
    const float phi =
        float(neighborCount) / float(globalSpeciesParams[sid].maxNeighbors);
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

    // -------------------------------------------------------
    // 5-B. Fast-start 吸引項の加算 (isAttracting==1 の間)
    // -------------------------------------------------------
    if (buf->isAttracting[gIdx]) {
      glm::vec3 dirSum(0.0f);
      int dirCnt = 0;

      const float reSq = globalSpeciesParams[sid].separationRange *
                         globalSpeciesParams[sid].separationRange;
      const float raSq = globalSpeciesParams[sid].cohesionRange *
                         globalSpeciesParams[sid].cohesionRange;

      for (size_t i = 0; i < indices.size(); ++i) {
        if (i == index)
          continue;
        glm::vec3 diff = buf->positions[indices[i]] - pos;
        float distSq = glm::dot(diff, diff);

        if (distSq > reSq && distSq <= raSq) {
          float dist = glm::sqrt(distSq);
          if (dist > EPS) {
            dirSum += diff / dist;
            ++dirCnt;
          }
        }
      }

      if (dirCnt > 0) {
        glm::vec3 avgDir = glm::normalize(dirSum / float(dirCnt));
        glm::vec3 desiredVel = avgDir * globalSpeciesParams[sid].maxSpeed;
        glm::vec3 attractAcc =
            (desiredVel - vel) * globalSpeciesParams[sid].lambda;
        buf->accelerations[gIdx] += attractAcc;
      }
    }

    if (neighborCount > 0) {
      glm::vec3 sumSep = glm::vec3(0.0f);
      glm::vec3 sumAlign = glm::vec3(0.0f);
      glm::vec3 sumCoh = glm::vec3(0.0f);
      float invN =
          1.0f /
          float(neighborCount); // activeNeighbors内の立っているビットを探索
      for (size_t i = 0; i < indices.size(); ++i) {
        if (!activeNeighbors.test(i))
          continue;

        if (cohesionMemories[i] <= 0.0f)
          continue;

        int gNeighbor = indices[i];
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (distSq <= 1e-4f)
          continue;

        float dist = glm::sqrt(distSq);
        float wSep = 1.0f - (dist / globalSpeciesParams[sid].separationRange);
        wSep = glm::clamp(wSep, 0.0f, 1.0f);
        sumSep += (diff * wSep) * (-1.0f);

        float wCoh = glm::clamp(dist / globalSpeciesParams[sid].cohesionRange,
                                0.0f, 1.0f);
        // stress に応じて凝集強度を増加
        float stressFactor = 1.0f + buf->stresses[gIdx] * 0.2f;
        wCoh *= stressFactor;

        sumCoh += buf->positions[gNeighbor] * wCoh;

        sumAlign += buf->velocities[gNeighbor];
      } // 分離の最終ベクトル
      glm::vec3 totalSeparation = glm::vec3(0.0f);
      float sepLen2 = glm::length2(sumSep);
      if (sepLen2 > EPS) {
        totalSeparation = (sumSep * (1.0f / glm::sqrt(sepLen2))) *
                          globalSpeciesParams[sid].separation;
      }

      // 凝集の最終ベクトル
      glm::vec3 avgCohPos = sumCoh * invN;
      glm::vec3 totalCohesion = glm::vec3(0.0f);
      glm::vec3 cohDir = avgCohPos - pos;
      float cohLen2 = glm::length2(cohDir);
      if (cohLen2 > EPS) {
        totalCohesion = (cohDir * (1.0f / glm::sqrt(cohLen2))) *
                        globalSpeciesParams[sid].cohesion;
      }

      // 整列の最終ベクトル
      glm::vec3 avgAlignVel = sumAlign * invN;
      glm::vec3 totalAlignment = glm::vec3(0.0f);
      glm::vec3 aliDir = avgAlignVel - vel;
      float aliLen2 = glm::length2(aliDir);
      if (aliLen2 > EPS) {
        totalAlignment = (aliDir * (1.0f / glm::sqrt(aliLen2))) *
                         globalSpeciesParams[sid].alignment;
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
    glm::vec3 avg = glm::vec3(0.0f);
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
    u->level = level + 1;
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
 * - 各子ノードの親/トップを設定
 * - 自身は中間ノードに変更
 */
void BoidUnit::splitInPlace(int maxBoids) {
  if (!needsSplit(80.0f, 0.5f, maxBoids))
    return;

  auto splits = splitByClustering(4);

  for (auto *child : splits) {
    child->parent = this;
    child->topParent = topParent;
    children.push_back(child);
  }

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
    std::vector<glm::vec3> newCenters(numClusters, glm::vec3(0.0f));
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
    u->level = level + 1;
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
 * 他ユニットを結合する（ポインタ／親ノード付き版）。
 *
 * 処理内容:
 * - indicesを結合
 * - speciesIdを更新
 * - 親とトップを設定
 * - 自ノードを葉に変更
 */
void BoidUnit::mergeWith(BoidUnit *other, BoidUnit *parent) {
  // indices を結合
  indices.insert(indices.end(), other->indices.begin(), other->indices.end());

  // buf->speciesIds を更新
  for (int gIdx : other->indices) {
    buf->speciesIds[gIdx] = speciesId; // 自ノードの speciesId を適用
  }
  // 親とトップを設定
  this->parent = parent;
  this->topParent = parent ? parent->topParent : this;

  // 自ノードを葉に戻す
  children.clear();

  // バウンディングスフィアを再計算
  computeBoundingSphere();

  // 親ノードから other を削除
  if (parent) {
    auto it =
        std::find(parent->children.begin(), parent->children.end(), other);
    if (it != parent->children.end())
      parent->children.erase(it);
  }

  // other を削除
  delete other;
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