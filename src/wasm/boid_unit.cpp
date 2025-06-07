#include "boid_unit.h"
#include "boids_tree.h"
#include <algorithm>
#include <future>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <stack>
#include <vector>

bool BoidUnit::isBoidUnit() const { return children.empty(); }

inline glm::vec3 approxRotate(const glm::vec3 &v, const glm::vec3 &axis,
                              float angle) {
  // 軽量化: 小角度近似をさらに簡略化
  // sinθ ≈ θ, cosθ ≈ 1 を前提に、外積計算を最小化
  constexpr float EPSILON = 1e-6f; // 極小値判定用
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
      return;

    // 子ノードの中心を計算
    center = glm::vec3(0.0f);
    for (const auto &child : children)
      center += child->center;
    center /= static_cast<float>(children.size());

    // 子ノード中心までの平均距離 + 子ノード半径
    float sum = 0.0f, sum2 = 0.0f;
    for (const auto &child : children) {
      float d = glm::distance(center, child->center) + child->radius;
      sum += d;
      sum2 += d * d;
    }
    float mean = sum / static_cast<float>(children.size());
    float var = sum2 / static_cast<float>(children.size()) - mean * mean;
    float stddev = var > 0.0f ? std::sqrt(var) : 0.0f;

    radius = mean + 1.0f * stddev;
  }
}

/**
 * 他のユニットとの相互作用を計算し、加速度に影響を加える。
 *
 * @param other 他の BoidUnit へのポインタ
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - Boid 間の詳細な相互作用を計算し、分離、整列、凝集ルールを適用。
 * - **片方が中間ノードの場合**:
 *   - 再帰的に子ノードを処理。
 * - **両方が中間ノードの場合**:
 *   - 代表値（中心、平均速度）を用いて近似計算を行う。
 *
 * 使用例:
 * - 階層構造内で異なるユニット間の影響を計算する際に使用。
 */
void BoidUnit::applyInterUnitInfluence(BoidUnit *other, float dt) {
  if (frameCount % 6 != 0) {
    // 再構築頻度でない場合は何もしない
    return;
  }
  if (!indices.empty() && !other->indices.empty()) {
    for (int idxA : indices) {

      glm::vec3 sumVel = glm::vec3(0.0f);
      glm::vec3 sumPos = glm::vec3(0.0f);
      glm::vec3 sep = glm::vec3(0.0f);
      int cnt = 0;

      for (int idxB : other->indices) {

        glm::vec3 diff = buf->positions[idxA] - other->buf->positions[idxB];
        float d2 = glm::dot(diff, diff);

        if (d2 < 2500.0f && d2 > 1e-4f) {
          float d = std::sqrt(d2);
          float w = std::max(0.0f, 1.0f - (d / 40.0f));

          sumVel += other->buf->velocities[idxB] * w;
          sumPos += other->buf->positions[idxB] * w;
          sep += (diff / (d2 + 1.0f)) * w;
          ++cnt;
        }
      }

      if (cnt > 0) {
        buf->accelerations[idxA] +=
            (sumVel / float(cnt) - buf->velocities[idxA]) *
            globalSpeciesParams.alignment;
        buf->accelerations[idxA] +=
            (sumPos / float(cnt) - buf->positions[idxA]) *
            (globalSpeciesParams.cohesion);
        buf->accelerations[idxA] +=
            sep * (globalSpeciesParams.separation * 0.5f);
      }
    }
  }
}
#include "pool_accessor.h"
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

    if (current->isBoidUnit()) {
      // Leaf は並列実行
      asyncTasks.emplace_back(
          pool.enqueue([current, dt] { current->computeBoidInteraction(dt); }));
    } else {
      // 中間ノードは逐次
      for (auto *child : current->children)
        stack.push(child);

      for (size_t a = 0; a < current->children.size(); ++a)
        for (size_t b = a + 1; b < current->children.size(); ++b)
          asyncTasks.emplace_back(pool.enqueue([current, a, b] {
            current->children[a]->applyInterUnitInfluence(current->children[b]);
          }));
      current->computeBoundingSphere();
    }
  }

  // ── ② ここで同期してから 2 段階目へ ─────────────────
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

        glm::vec3 velocity = current->buf->velocities[gIdx];
        glm::vec3 acceleration = current->buf->accelerations[gIdx];
        glm::vec3 desiredVelocity = velocity + acceleration * dt;

        glm::vec3 oldDir = glm::normalize(velocity);
        glm::vec3 newDir = glm::normalize(desiredVelocity);
        float speed = glm::length(desiredVelocity);

        float dotProduct = glm::dot(oldDir, newDir);
        float angle = acosf(glm::clamp(dotProduct, -1.0f, 1.0f));

        if (angle > globalSpeciesParams.maxTurnAngle) {
          glm::vec3 axis = glm::cross(oldDir, newDir);
          float axisLength2 = glm::length2(axis);
          if (axisLength2 > 1e-8f) {
            axis /= glm::sqrt(axisLength2); // 正規化
            float rot = glm::min(angle, globalSpeciesParams.maxTurnAngle * dt);
            newDir = approxRotate(oldDir, axis, rot);
          }
        }
        float tilt = newDir.y;
        if (fabsf(tilt) > 1e-4f) {
          glm::vec3 flatDir = glm::normalize(glm::vec3(newDir.x, 0, newDir.z));
          glm::vec3 axis = glm::cross(newDir, flatDir);
          float flatAngle =
              acosf(glm::clamp(glm::dot(newDir, flatDir), -1.0f, 1.0f));
          float axisLength2 = glm::length2(axis);
          if (flatAngle > 1e-4f && axisLength2 > 1e-8f) {
            axis /= glm::sqrt(axisLength2); // 正規化
            float rot =
                glm::min(flatAngle, globalSpeciesParams.horizontalTorque * dt);
            newDir = approxRotate(newDir, axis, rot);
          }
        }

        float finalSpeed = glm::clamp(speed, globalSpeciesParams.minSpeed,
                                      globalSpeciesParams.maxSpeed);

        current->buf->velocities[gIdx] = newDir * finalSpeed;
        current->buf->positions[gIdx] += current->buf->velocities[gIdx] * dt;
        current->buf->accelerations[gIdx] = glm::vec3(0.0f);
        // クォータニオンを計算して orientations に格納
        current->buf->orientations[gIdx] = BoidUnit::dirToQuatRollZero(newDir);
      }
    } else {
      for (auto &child : current->children)
        stack.push(child);
    }
  }
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

void BoidUnit::computeBoidInteraction(float dt) {
  // ■ 各 Boid に対する一時変数（繰り返し再利用）
  glm::vec3 separation;
  glm::vec3 alignment;
  glm::vec3 cohesion;

  int gIdx = 0;
  glm::vec3 pos;
  glm::vec3 vel;

  // -----------------------------------------------
  // 事前計算しておく定数／準備
  // -----------------------------------------------
  // ① 視界範囲 (cohesionRange) の二乗
  const float viewRangeSq =
      globalSpeciesParams.cohesionRange * globalSpeciesParams.cohesionRange;
  // ② 視界角度（FOV in degrees）の半分をラジアンに変換 & そのコサイン
  float halfFovRad = glm::radians(globalSpeciesParams.fieldOfViewDeg * 0.5f);
  float cosHalfFov = std::cos(halfFovRad);

  // ③ 非ゼロ判定用イプシロン
  constexpr float EPS = 1e-8f;

  // ④ 候補距離を入れてソート/部分ソートするための領域
  std::vector<std::pair<float, int>> candidates;
  if (candidates.capacity() < indices.size()) {
    candidates.reserve(indices.size());
  }

  // -----------------------------------------------
  // 各 Boid（leafノード内）ごとの反復
  // -----------------------------------------------
  for (size_t index = 0; index < indices.size(); ++index) {
    // -------------------------------------------------------
    // ❶ 初期化フェーズ
    //    ・加速度計算用に separation/alignment/cohesion をリセット
    //    ・対象 Boid のグローバルインデックスと位置・速度を取得
    // -------------------------------------------------------
    separation = glm::vec3(0.00001f);
    alignment = glm::vec3(0.00001f);
    cohesion = glm::vec3(0.00001f);

    gIdx = indices[index];
    pos = buf->positions[gIdx];
    vel = buf->velocities[gIdx];

    // 「距離候補リスト」は毎回クリア
    candidates.clear();

    // -------------------------------------------------------
    // ❷ 時間更新＆古くなった記憶の無効化
    //    ・cohesionMemories[i] > 0 のものは時間を加算
    //    ・tau を超えたら 0 に戻して、ビットをクリア
    //    ・activeCount には有効な隣接 Boid 数を数える
    // -------------------------------------------------------
    int activeCount = 0;
    for (size_t i = 0; i < indices.size(); ++i) {
      if (cohesionMemories[i] > 0.0f) {
        cohesionMemories[i] += dt;
        if (cohesionMemories[i] > globalSpeciesParams.tau) {
          cohesionMemories[i] = 0.0f;
          activeNeighbors.reset(i);
        } else {
          activeCount++;
        }
      }
    }

    // -------------------------------------------------------
    // ❸ 未登録の Boid で「最も近い (距離かつ視界内)」ものを
    //     上位 toAdd 件分見つける
    //    ・activeCount < maxNeighbors のときだけ行う
    //    ・距離判定: distSq < viewRangeSq
    //    ・視界判定: (速度方向がほぼゼロでなければ)
    //    normalized(diff)·normalized(vel) >= cosHalfFov
    // -------------------------------------------------------
    if (activeCount < globalSpeciesParams.maxNeighbors) {
      // — 速度ベクトル vel がほぼゼロかどうかチェック
      float velLen2 = glm::length2(vel);

      bool hasVel = (velLen2 > EPS);
      glm::vec3 forward;
      if (hasVel) {
        float invVelLen = 1.0f / glm::sqrt(velLen2);
        forward = vel * invVelLen; // 速度方向の正規化ベクトル
      }

      for (size_t i = 0; i < indices.size(); ++i) {
        if (i == index)
          continue; // 自分自身をスキップ
        if (activeNeighbors.test(i) || cohesionMemories[i] > 0.0f)
          continue; // 登録済み or 有効な隣接Boidはスキップ

        int gNeighbor = indices[i];
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (distSq >= viewRangeSq)
          continue; // 視界/距離外ならスキップ

        // — 速度ゼロでなければ視界内かどうかを確認
        if (hasVel) {
          float invDist = 1.0f / glm::sqrt(distSq);
          glm::vec3 diffNorm = diff * invDist;
          float dotVal = glm::dot(forward, diffNorm);
          if (dotVal < cosHalfFov)
            continue; // 視界外 → スキップ
        }

        // 条件をすべてクリアしたので候補リストに追加（距離², ローカル index）
        candidates.emplace_back(distSq, (int)i);
      }
    }

    // -------------------------------------------------------
    // ❹ 候補リストの中から最も近い toAdd 件を選んで登録
    //    ・toAdd = maxNeighbors - activeCount
    //    ・部分ソート (nth_element) して上位 toAdd 件だけ取り出す
    // -------------------------------------------------------
    int toAdd = globalSpeciesParams.maxNeighbors - activeNeighbors.count();
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
        // 候補数 <= toAdd の場合は全件登録
        for (auto &pr : candidates) {
          int idx2 = pr.second;
          cohesionMemories[idx2] = dt;
          activeNeighbors.set(idx2);
        }
      }
    }

    // -------------------------------------------------------
    // ❺ 「有効な Boid」だけで最終的な加速度を計算
    //    ・activeNeighbors.test(i)==true の i についてのみ
    //    ・分離 (separation)、凝集 (cohesion)、整列 (alignment) の Contribution
    //    を合算 ・最後に回転トルク (alignment を向くための補正)
    //    を行い加速度に加える
    // -------------------------------------------------------
    // -------------------------------------------------------
    // ❺-A  近傍不足 Fast-start 吸引制御  (φᵢ = |Lᵢ| / Nu)
    //        ・φᵢ < 1 なら吸引 ON, タイマー τ をリセット
    //        ・φᵢ = 1 なら τ カウントダウン → 0 で OFF
    // -------------------------------------------------------
    const int neighborCount = static_cast<int>(activeNeighbors.count());
    const float phi =
        float(neighborCount) / float(globalSpeciesParams.maxNeighbors);

    if (phi < 1.0f) {
      // 群れの縁に出た ⇒ 吸引 ON & タイマー毎フレーム再スタート
      buf->isAttracting[gIdx] = 1;
      buf->attractTimers[gIdx] = globalSpeciesParams.tau;
    } else if (buf->isAttracting[gIdx]) {
      // 内部に戻っていれば τ カウントダウン
      buf->attractTimers[gIdx] -= dt;
      if (buf->attractTimers[gIdx] <= 0.0f) {
        buf->isAttracting[gIdx] = 0;
        buf->attractTimers[gIdx] = 0.0f;
      }
    }

    // -------------------------------------------------------
    // ❺-B  Fast-start 吸引項の加算 (isAttracting==1 の間)
    // -------------------------------------------------------
    if (buf->isAttracting[gIdx]) {
      glm::vec3 dirSum(0.0f);
      int dirCnt = 0;

      const float reSq = globalSpeciesParams.separationRange *
                         globalSpeciesParams.separationRange;
      const float raSq =
          globalSpeciesParams.cohesionRange * globalSpeciesParams.cohesionRange;

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
        glm::vec3 desiredVel = avgDir * globalSpeciesParams.maxSpeed;
        glm::vec3 attractAcc = (desiredVel - vel) * globalSpeciesParams.lambda;
        buf->accelerations[gIdx] += attractAcc;
      }
    }

    if (neighborCount > 0) {
      glm::vec3 sumSep = glm::vec3(0.0f);
      glm::vec3 sumAlign = glm::vec3(0.0f);
      glm::vec3 sumCoh = glm::vec3(0.0f);
      float invN = 1.0f / float(neighborCount);

      // activeNeighbors 内の「立っているビット」を単純ループで探索
      for (size_t i = 0; i < indices.size(); ++i) {
        if (!activeNeighbors.test(i))
          continue;

        if (cohesionMemories[i] <= 0.0f)
          continue; // 念のためガード

        int gNeighbor = indices[i];
        glm::vec3 diff = buf->positions[gNeighbor] - pos;
        float distSq = glm::dot(diff, diff);
        if (distSq <= 1e-4f)
          continue; // ほぼ同一位置ならスキップ

        float dist = glm::sqrt(distSq);
        float wSep = 1.0f - (dist / globalSpeciesParams.separationRange);
        wSep = glm::clamp(wSep, 0.0f, 1.0f);
        sumSep += (diff * wSep) * (-1.0f); // 分離

        float wCoh =
            glm::clamp(dist / globalSpeciesParams.cohesionRange, 0.0f, 1.0f);
        sumCoh += buf->positions[gNeighbor] * wCoh; // 凝集

        sumAlign += buf->velocities[gNeighbor]; // 整列
      }

      // --- 分離の最終ベクトル ---
      glm::vec3 totalSeparation = glm::vec3(0.0f);
      float sepLen2 = glm::length2(sumSep);
      if (sepLen2 > EPS) {
        totalSeparation = (sumSep * (1.0f / glm::sqrt(sepLen2))) *
                          globalSpeciesParams.separation;
      }

      // --- 凝集の最終ベクトル ---
      glm::vec3 avgCohPos = sumCoh * invN;
      glm::vec3 totalCohesion = glm::vec3(0.0f);
      glm::vec3 cohDir = avgCohPos - pos;
      float cohLen2 = glm::length2(cohDir);
      if (cohLen2 > EPS) {
        totalCohesion = (cohDir * (1.0f / glm::sqrt(cohLen2))) *
                        globalSpeciesParams.cohesion;
      }

      // --- 整列の最終ベクトル ---
      glm::vec3 avgAlignVel = sumAlign * invN;
      glm::vec3 totalAlignment = glm::vec3(0.0f);
      glm::vec3 aliDir = avgAlignVel - vel;
      float aliLen2 = glm::length2(aliDir);
      if (aliLen2 > EPS) {
        totalAlignment = (aliDir * (1.0f / glm::sqrt(aliLen2))) *
                         globalSpeciesParams.alignment;
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
                  std::min(ang2, globalSpeciesParams.torqueStrength * dt);
              rot2 = std::min(rot2, globalSpeciesParams.maxTurnAngle);
              glm::vec3 newDir2 = approxRotate(forward2, axis2, rot2);

              // 速度ベクトルを回転後の方向に更新
              vel = newDir2 * glm::length(vel);

              // 加速度にもトルク分を加算
              buf->accelerations[gIdx] +=
                  axis2 * ang2 * globalSpeciesParams.torqueStrength;
            }
          }
        }
      }

      // --- 最終的な加速度をバッファに書き込み ---
      buf->accelerations[gIdx] +=
          totalSeparation + totalAlignment + totalCohesion;
    }
  }
}

// 分割が必要か判定 (indices + 中央バッファ版)
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

// 最大分散軸で分割 → 任意分割 (indices 版)
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

// 現在のユニットを分割し子ノードとして配置
void BoidUnit::splitInPlace(int maxBoids) {
  if (!needsSplit(80.0f, 0.5f, maxBoids))
    return;

  auto splits = splitByClustering(4);

  // 自分は中間ノードになるので indices を空に
  indices.clear();

  // 子ノードを登録
  children = std::move(splits);

  // バウンディングスフィアを再計算
  computeBoundingSphere();
}
// k-means 風クラスタリングで indices をグループ化
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
    u->computeBoundingSphere();
    result.push_back(u);
  }
  return result;
}

/**
 * 指定されたユニット内のすべての Boid に反発力を加える。
 *
 * @param unit 対象の BoidUnit
 * @param repulsion 反発力ベクトル
 *
 * 処理内容:
 * - **最下位層の場合**:
 *   - Boid の位置に基づいて反発力を計算し、加速度に加算。
 * - **中間ノードの場合**:
 *   - 再帰的に子ノードを処理。
 *
 * 使用例:
 * - Boid 間の衝突回避やユニット間の分離を実現する際に使用。
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

// 他ユニットを結合（値渡し版）
void BoidUnit::mergeWith(const BoidUnit &other) {
  indices.insert(indices.end(), other.indices.begin(), other.indices.end());
  computeBoundingSphere();
}

// 他ユニットを結合（ポインタ／親ノード付き版）
void BoidUnit::mergeWith(BoidUnit *other, BoidUnit *parent) {
  indices.insert(indices.end(), other->indices.begin(), other->indices.end());
  children.clear(); // 自ノードを葉に戻す
  computeBoundingSphere();

  if (parent) {
    auto it =
        std::find(parent->children.begin(), parent->children.end(), other);
    if (it != parent->children.end())
      parent->children.erase(it);
  }
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