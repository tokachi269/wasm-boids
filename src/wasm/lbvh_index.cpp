#include "lbvh_index.h"

// Boid 群の近傍探索を支える LBVH (Linear BVH) を実装する。目的は
// 「毎フレームの再構築コストを抑えつつ、k-NN と半径クエリを高速に返すこと」。
// build() は Morton Order を用いた線形レイアウトで静的木を生成し、refit() が葉ノード
// の AABB を最新位置に追従させる。gatherNearest() は木全体を走査し、最も近い Boid を
// 固定長ヒープに蓄積した上で距離昇順に返す。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

// Morton コード生成のヘルパー関数群。
// 3D 座標 (x,y,z) を単一整数にマッピングし、空間的に近い点が数値的にも近くなる
// よう Z-order curve 上に配置する。これにより線形ソートだけで空間局所性を得られる。

inline uint32_t expandBits(uint32_t v) {
  // 10bit 値を 30bit に展開（2bit おきに配置）
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

inline uint32_t morton3D(uint32_t x, uint32_t y, uint32_t z) {
  // 各軸を 2bit 間隔で交互配置し、Z-order インデックスを生成
  return (expandBits(x) << 2) | (expandBits(y) << 1) | expandBits(z);
}

// 木構築時に Morton コードでソートするための一時構造体
struct Primitive {
  uint32_t morton = 0;  // Z-order 値
  int index = 0;        // 元の Boid インデックス
};

} // namespace

LbvhIndex::LbvhIndex(int maxLeafSize)
  : maxLeafSize_(std::max(8, maxLeafSize)) {}

// SoA バッファを元に LBVH 木を一から構築する。毎フレーム呼ぶのではなく
// 大幅なレイアウト変更時のみ実行する想定。通常は refit() で葉 AABB を更新するだけ。
void LbvhIndex::build(const SoABuffers &buffers) {
  buffers_ = &buffers;

  const std::size_t count = buffers.positions.size();
  // 既存データをクリアして新しい木を準備
  nodes_.clear();
  leafRecords_.clear();
  leafIndexStorage_.clear();
  mortonOrder_.clear();
  boidToLeafIndex_.clear();

  if (count == 0) {
    return;  // 空データなら何もしない
  }

  boidToLeafIndex_.assign(count, -1);

  // 全 Boid の AABB を計算。Morton コード生成で正規化するため必要。
  glm::vec3 boundsMin(std::numeric_limits<float>::max());
  glm::vec3 boundsMax(-std::numeric_limits<float>::max());
  for (const auto &pos : buffers.positions) {
    boundsMin = vecMin(boundsMin, pos);
    boundsMax = vecMax(boundsMax, pos);
  }

  // 各軸の範囲がゼロに近い場合は 1.0 に拡張（ゼロ除算回避）
  glm::vec3 extent = boundsMax - boundsMin;
  extent.x = extent.x <= 1e-6f ? 1.0f : extent.x;
  extent.y = extent.y <= 1e-6f ? 1.0f : extent.y;
  extent.z = extent.z <= 1e-6f ? 1.0f : extent.z;

  std::vector<Primitive> primitives;
  primitives.reserve(count);

  // 各 Boid 位置を [0,1]³ に正規化してから Morton コードを算出
  for (std::size_t i = 0; i < count; ++i) {
    const glm::vec3 normalized = (buffers.positions[i] - boundsMin) / extent;
    const glm::vec3 clamped = glm::clamp(normalized, glm::vec3(0.0f),
                                         glm::vec3(0.999999f));
    constexpr float kScale = 1024.0f; // 各軸 10bit = 計 30bit の解像度
    const uint32_t ix = static_cast<uint32_t>(clamped.x * kScale);
    const uint32_t iy = static_cast<uint32_t>(clamped.y * kScale);
    const uint32_t iz = static_cast<uint32_t>(clamped.z * kScale);
    primitives.push_back(Primitive{morton3D(ix, iy, iz), static_cast<int>(i)});
  }

  // Morton コード順にソート（同値の場合は元インデックス順で安定化）
  std::sort(primitives.begin(), primitives.end(),
            [](const Primitive &a, const Primitive &b) {
              if (a.morton == b.morton)
                return a.index < b.index; // 安定ソート化
              return a.morton < b.morton;
            });

  // ソート済み順序で Boid インデックス配列を構築
  mortonOrder_.reserve(count);
  for (const auto &prim : primitives) {
    mortonOrder_.push_back(prim.index);
  }

  // 二分木なので最大 2*count 個のノードを確保してから再帰構築開始
  nodes_.reserve(static_cast<std::size_t>(2 * count));
  buildRecursive(0, static_cast<int>(count));
}

// 既存の木構造を保ったまま、全ノードの AABB を最新の Boid 位置に追従させる。
// 毎フレーム実行する想定で、build() より遥かに軽量。葉→親の順で AABB を更新し
// クエリ精度を維持する。
void LbvhIndex::refit(const SoABuffers &buffers) {
  buffers_ = &buffers;

  if (!buffers_ || nodes_.empty() || leafRecords_.empty()) {
    return; // 木が未構築または空データなら何もしない
  }

  const glm::vec3 *positions = buffers_->positions.data();
  if (!positions) {
    return;
  }

  // 第一段階: 各葉ノードの AABB を所属 Boid の現在位置から再計算
  for (const LeafRecord &record : leafRecords_) {
    Node &leafNode = nodes_[record.nodeIndex];
    glm::vec3 boundsMin(std::numeric_limits<float>::max());
    glm::vec3 boundsMax(-std::numeric_limits<float>::max());
    const int leafEnd = record.offset + record.count;
    bool anyValid = false;
    // 葉内の全 Boid を走査して AABB を拡張
    for (int i = record.offset; i < leafEnd; ++i) {
      const int boidIdx = leafIndexStorage_[i];
      if (boidIdx < 0 ||
          boidIdx >= static_cast<int>(buffers_->positions.size())) {
        continue; // 不正インデックスはスキップ
      }
      const glm::vec3 &pos = positions[boidIdx];
      boundsMin = vecMin(boundsMin, pos);
      boundsMax = vecMax(boundsMax, pos);
      anyValid = true;
    }
    if (!anyValid) {
      // 有効な Boid がない場合はゼロサイズ AABB
      boundsMin = glm::vec3(0.0f);
      boundsMax = glm::vec3(0.0f);
    }
    leafNode.boundsMin = boundsMin;
    leafNode.boundsMax = boundsMax;
  }

  // 第二段階: 葉→根に向けて内部ノードの AABB を子の結合で更新
  // 後ろ（葉に近い）から前（根に近い）への逆順走査で依存関係を満たす
  for (int nodeIndex = static_cast<int>(nodes_.size()) - 1; nodeIndex >= 0;
       --nodeIndex) {
    Node &node = nodes_[nodeIndex];
    if (node.isLeaf) {
      continue; // 葉は既に更新済み
    }
    // 左右子ノードの AABB を結合
    const Node &left = nodes_[node.left];
    const Node &right = nodes_[node.right];
    node.boundsMin = vecMin(left.boundsMin, right.boundsMin);
    node.boundsMax = vecMax(left.boundsMax, right.boundsMax);
  }
}

// 全ての葉ノードを走査し、各葉に含まれる Boid インデックス配列を visitor に渡す。
// デバッグ用途や統計情報収集に使われる想定。
void LbvhIndex::forEachLeaf(const LeafVisitor &visitor) const {
  if (!visitor)
    return; // visitor が null なら何もしない
  for (const auto &leaf : leafRecords_) {
    const int *ptr = leafIndexStorage_.data() + leaf.offset;
    visitor(SpatialLeaf{ptr, static_cast<std::size_t>(leaf.count), nullptr});
  }
}

// 指定した球（中心・半径）と交差する葉ノードのみを走査する半径クエリ。
// 木を深さ優先で辿り、各ノードの AABB が球と交差するかを判定して枝刈りする。
// 葉ノードに到達したら visitor を呼び出して Boid インデックス配列を渡す。
void LbvhIndex::forEachLeafIntersectingSphere(
    const glm::vec3 &center, float radius, const LeafVisitor &visitor) const {
  if (!visitor || nodes_.empty())
    return; // visitor が null または木が空なら何もしない

  const float radiusSq = radius * radius;
  std::vector<int> stack;
  stack.reserve(64); // 深さ優先探索用スタック
  stack.push_back(0); // 根ノードから開始

  while (!stack.empty()) {
    const int nodeIndex = stack.back();
    stack.pop_back();
    const Node &node = nodes_[nodeIndex];

    // 中心点を AABB に clamp して最短距離を計算（球-AABB 交差判定）
    const glm::vec3 clamped = glm::clamp(center, node.boundsMin, node.boundsMax);
    const glm::vec3 diff = clamped - center;
    const float distSq = glm::dot(diff, diff);
    if (distSq > radiusSq)
      continue; // 球と交差しないノードは枝刈り

    if (node.isLeaf) {
      // 葉ノードなら visitor に Boid インデックス配列を渡す
      const int *ptr = leafIndexStorage_.data() + node.begin;
      visitor(SpatialLeaf{ptr,
                          static_cast<std::size_t>(node.end - node.begin),
                          nullptr});
    } else {
      // 内部ノードなら子ノードをスタックに積む
      if (node.left >= 0)
        stack.push_back(node.left);
      if (node.right >= 0)
        stack.push_back(node.right);
    }
  }
}

namespace {

// 点と AABB（軸並行バウンディングボックス）間の最短距離の二乗を計算。
// 点が AABB 内部にある場合は 0 を返す。外部にある場合は最短面までの距離²。
float distanceToAabbSq(const glm::vec3 &point, const glm::vec3 &minBounds,
                       const glm::vec3 &maxBounds) {
  const float dx = std::max(std::max(minBounds.x - point.x, 0.0f),
                            point.x - maxBounds.x);
  const float dy = std::max(std::max(minBounds.y - point.y, 0.0f),
                            point.y - maxBounds.y);
  const float dz = std::max(std::max(minBounds.z - point.z, 0.0f),
                            point.z - maxBounds.z);
  return dx * dx + dy * dy + dz * dz;
}

} // namespace

// 指定中心点から距離が近い Boid を効率的に探索する k-NN（k 近傍）クエリの実装。
// 
// アルゴリズム概要:
// 1. 木をスタックベースの深さ優先で走査（近い子ノードから処理）
// 2. 葉ノードでは全 Boid の実距離を計算し、候補ヒープ best に蓄積
// 3. best が満杯になるにつれ currentWorstSq が縮小し、枝刈り効率が向上
// 4. 最終的に best を距離昇順にソートして返却
//
// 最適化要素:
// - 距離計算の段階的早期終了（x → xy → xyz の順で閾値判定）
// - near-first 探索によるヒープ満杯までの高速化
// - スレッドローカル作業バッファの再利用
int LbvhIndex::gatherNearest(const glm::vec3 &center, int maxCount,
                             float maxRadius, int *outIndices,
                             float *outDistancesSq) const {
  if (maxCount <= 0 || !outIndices || !outDistancesSq || nodes_.empty() ||
      !buffers_ || buffers_->positions.empty()) {
    return 0;
  }

  stats_.queries.fetch_add(1, std::memory_order_relaxed);

  int localNodesVisited = 0;
  int localLeavesVisited = 0;
  int localBoidsConsidered = 0;
  int localMaxQueue = 0;

  const float maxRadiusSq = (maxRadius > 0.0f)
                                ? (maxRadius * maxRadius)
                                : std::numeric_limits<float>::infinity();
  const bool hasRadiusLimit = std::isfinite(maxRadiusSq);

  auto &scratch = threadScratch();
  auto &stack = scratch.stack;
  stack.clear();

  constexpr int kTraversalStackReserve = 192; // 深さ優先探索に十分な初期容量
  if (static_cast<int>(stack.capacity()) < kTraversalStackReserve) {
    stack.reserve(kTraversalStackReserve);
  }

  const float rootDist =
      distanceToAabbSq(center, nodes_[0].boundsMin, nodes_[0].boundsMax);
  if (hasRadiusLimit && rootDist > maxRadiusSq) {
    return 0;
  }

  const auto updateStackSize = [&]() {
    const int currentSize = static_cast<int>(stack.size());
    if (currentSize > localMaxQueue) {
      localMaxQueue = currentSize; // 統計用：最大スタック深度を記録
    }
  };
  // 根ノードから探索開始
  stack.push_back(QueueEntry{0, rootDist});
  updateStackSize();

  auto &best = scratch.best;
  best.clear();
  if (best.capacity() < static_cast<std::size_t>(maxCount)) {
    best.reserve(static_cast<std::size_t>(maxCount));
  }
  float currentWorstSq = maxRadiusSq;

  // 候補ヒープの状況に応じて枝刈り閾値 currentWorstSq を更新
  auto updateWorst = [&]() {
    if (static_cast<int>(best.size()) < maxCount) {
      currentWorstSq = maxRadiusSq; // まだ満杯でなければ半径制限のみ
    } else if (!best.empty()) {
      // 満杯なら最遠候補と半径制限の小さい方を閾値とする
      currentWorstSq = std::min(maxRadiusSq, best.front().first);
    }
  };

  const auto heapComp = [](const auto &a, const auto &b) {
    return a.first < b.first; // first が距離²。最大ヒープを維持する比較。
  };

  // best は「これまで見つかった最短 maxCount 件」を保持する固定長ヒープ。
  // pushCandidate はサイズ不足時に単に挿入し、満杯なら最遠要素より近い場合のみ
  // 差し替える。currentWorstSq を更新しておくことで木探索側の枝刈り効率が上がる。
  auto pushCandidate = [&](float distSq, int index) {
    if (static_cast<int>(best.size()) < maxCount) {
      best.emplace_back(distSq, index);
      std::push_heap(best.begin(), best.end(), heapComp);
      updateWorst();
    } else if (!best.empty() && distSq < best.front().first) {
      std::pop_heap(best.begin(), best.end(), heapComp);
      best.back() = std::make_pair(distSq, index);
      std::push_heap(best.begin(), best.end(), heapComp);
      updateWorst();
    }
  };

  const glm::vec3 *positions = buffers_->positions.data();
  const float centerX = center.x;
  const float centerY = center.y;
  const float centerZ = center.z;

  const auto radiusAllows = [&](float distSq) {
    return !hasRadiusLimit || distSq <= maxRadiusSq;
  };

  // メインループ: スタックが空になるまで深さ優先で木を辿る
  while (!stack.empty()) {
    const QueueEntry entry = stack.back();
    stack.pop_back();

    const float nodeDistSq = entry.distSq;
    // 現在の最悪候補より遠い、または半径制限を超えるノードは枝刈り
    if (nodeDistSq > currentWorstSq || !radiusAllows(nodeDistSq)) {
      continue;
    }

    ++localNodesVisited;
    const Node &node = nodes_[entry.nodeIndex];
    if (node.isLeaf) {
      ++localLeavesVisited;
      const int *leafIndices = leafIndexStorage_.data() + node.begin;
      const int leafCount = node.end - node.begin;
      for (int i = 0; i < leafCount; ++i) {
        const int boidIdx = leafIndices[i];
        ++localBoidsConsidered;
        const glm::vec3 &pos = positions[boidIdx];
        const float dx = pos.x - centerX;
        float distSq = dx * dx;
        if (distSq > currentWorstSq) {
          continue;
        }

        const float dy = pos.y - centerY;
        distSq += dy * dy;
        if (distSq > currentWorstSq) {
          continue;
        }

        const float dz = pos.z - centerZ;
        distSq += dz * dz;
        if (distSq > currentWorstSq || !radiusAllows(distSq)) {
          continue;
        }
        // 3 軸それぞれで currentWorstSq を超えたらすぐにスキップする。距離二乗の全計算と
        // pushCandidate 呼び出しを省略できるため、密な葉での無駄な浮動小数演算を削減できる。
        pushCandidate(distSq, boidIdx);
      }
      continue;
    }

    // 内部ノードの場合: 左右子への距離を計算し near/far を決定
    int leftIndex = node.left;
    int rightIndex = node.right;
    float leftDist = std::numeric_limits<float>::infinity();
    float rightDist = std::numeric_limits<float>::infinity();

    if (leftIndex >= 0) {
      leftDist = distanceToAabbSq(center, nodes_[leftIndex].boundsMin,
                                  nodes_[leftIndex].boundsMax);
    }
    if (rightIndex >= 0) {
      rightDist = distanceToAabbSq(center, nodes_[rightIndex].boundsMin,
                                   nodes_[rightIndex].boundsMax);
    }

    // 距離の小さい方を near、大きい方を far に分類
    int nearIndex = leftIndex;
    float nearDist = leftDist;
    int farIndex = rightIndex;
    float farDist = rightDist;
    if (rightDist < leftDist) {
      nearIndex = rightIndex;
      nearDist = rightDist;
      farIndex = leftIndex;
      farDist = leftDist;
    }

    const bool nearAllowed =
        nearIndex >= 0 && nearDist <= currentWorstSq && radiusAllows(nearDist);
    const bool farAllowed =
        farIndex >= 0 && farDist <= currentWorstSq && radiusAllows(farDist);

  // 遠方ノードはスタックに先に積むことで（LIFO の特性により）近距離ノードを先に
  // 展開できる。これにより currentWorstSq が速く縮み、遠距離ノードの枝刈りが容易に
  // なる。どちらも条件を満たさない場合は push しない。
    if (farAllowed) {
      stack.push_back(QueueEntry{farIndex, farDist});
      updateStackSize();
    }
    if (nearAllowed) {
      stack.push_back(QueueEntry{nearIndex, nearDist});
      updateStackSize();
    }
  }

  // 探索で収集した統計情報をグローバル統計に加算（マルチスレッド対応）
  stats_.nodesVisited.fetch_add(localNodesVisited, std::memory_order_relaxed);
  stats_.leavesVisited.fetch_add(localLeavesVisited,
                                 std::memory_order_relaxed);
  stats_.boidsConsidered.fetch_add(localBoidsConsidered,
                                   std::memory_order_relaxed);
  // maxQueueSize は最大値なので compare_exchange で更新
  int prevMax = stats_.maxQueueSize.load(std::memory_order_relaxed);
  while (localMaxQueue > prevMax &&
         !stats_.maxQueueSize.compare_exchange_weak(
             prevMax, localMaxQueue, std::memory_order_relaxed)) {
  }

  // 候補を距離昇順にソートして出力配列にコピー
  std::sort(best.begin(), best.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });
  const int count = std::min(maxCount, static_cast<int>(best.size()));
  for (int i = 0; i < count; ++i) {
    outDistancesSq[i] = best[i].first;  // 距離²
    outIndices[i] = best[i].second;     // Boid インデックス
  }
  return count; // 実際に見つかった件数を返す
}

// 指定した Boid が所属する葉ノード内の全メンバー（Boid インデックス配列）を取得。
// 同一葉に属する近傍を高速に列挙したい場合に使用する。戻り値 true なら
// outIndices に配列の先頭ポインタ、outCount に要素数が設定される。
bool LbvhIndex::getLeafMembers(int boidIndex, const int *&outIndices,
                               int &outCount) const {
  outIndices = nullptr;
  outCount = 0;

  // 引数の妥当性チェック
  if (!buffers_ || boidIndex < 0 ||
      boidIndex >= static_cast<int>(boidToLeafIndex_.size())) {
    return false;
  }

  const int leafIndex = boidToLeafIndex_[boidIndex];
  if (leafIndex < 0 || leafIndex >= static_cast<int>(leafRecords_.size())) {
    return false; // 対応する葉が見つからない
  }

  const LeafRecord &record = leafRecords_[leafIndex];
  if (record.count <= 0) {
    return false; // 葉が空
  }

  // 葉内の連続した Boid インデックス配列を出力パラメータに設定
  outIndices = leafIndexStorage_.data() + record.offset;
  outCount = record.count;
  return true;
}

// Morton ソート済みの Boid 配列 [start, end) に対して再帰的に二分木を構築。
// maxLeafSize_ 以下なら葉ノードを作り、それを超えるなら中点で分割して再帰する。
// 葉ノードでは所属 Boid の AABB を計算し、内部ノードでは子ノードの AABB を結合する。
int LbvhIndex::buildRecursive(int start, int end) {
  Node node;
  const int nodeIndex = static_cast<int>(nodes_.size());
  nodes_.push_back(node);

  Node &current = nodes_.back();

  const int count = end - start;
  if (count <= maxLeafSize_) {
    // 葉ノードを作成: 範囲内の全 Boid を格納し AABB を計算
    current.isLeaf = true;
    current.begin = static_cast<int>(leafIndexStorage_.size());
    current.end = current.begin + count;
    glm::vec3 boundsMin(std::numeric_limits<float>::max());
    glm::vec3 boundsMax(-std::numeric_limits<float>::max());
    for (int i = start; i < end; ++i) {
      const int boidIdx = mortonOrder_[i];
      leafIndexStorage_.push_back(boidIdx);
      const glm::vec3 &pos = buffers_->positions[boidIdx];
      boundsMin = vecMin(boundsMin, pos);
      boundsMax = vecMax(boundsMax, pos);
    }
    current.boundsMin = boundsMin;
    current.boundsMax = boundsMax;
    
    // 葉ノード記録と逆引きマッピングを構築
    const int recordIndex = static_cast<int>(leafRecords_.size());
    leafRecords_.push_back(LeafRecord{nodeIndex, current.begin, count});
    for (int i = current.begin; i < current.end; ++i) {
      const int boidIdx = leafIndexStorage_[i];
      if (boidIdx >= 0 &&
          boidIdx < static_cast<int>(boidToLeafIndex_.size())) {
        boidToLeafIndex_[boidIdx] = recordIndex; // Boid → 葉レコードのマッピング
      }
    }
    return nodeIndex;
  }

  // 内部ノードを作成: 中点で分割して左右の子を再帰構築
  const int mid = (start + end) / 2;
  current.left = buildRecursive(start, mid);
  current.right = buildRecursive(mid, end);
  current.isLeaf = false;

  // 左右子ノードの AABB を結合して自身の AABB とする
  current.boundsMin = vecMin(nodes_[current.left].boundsMin,
                             nodes_[current.right].boundsMin);
  current.boundsMax = vecMax(nodes_[current.left].boundsMax,
                             nodes_[current.right].boundsMax);

  return nodeIndex;
}

// ベクトル要素ごとの最小値を計算（AABB の min 頂点算出用）
glm::vec3 LbvhIndex::vecMin(const glm::vec3 &a, const glm::vec3 &b) {
  return glm::vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

// ベクトル要素ごとの最大値を計算（AABB の max 頂点算出用）
glm::vec3 LbvhIndex::vecMax(const glm::vec3 &a, const glm::vec3 &b) {
  return glm::vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

// 統計情報をゼロリセット。フレーム開始時に呼んでクエリ性能を測定する準備をする。
void LbvhIndex::resetQueryStats() const {
  stats_.queries.store(0, std::memory_order_relaxed);
  stats_.nodesVisited.store(0, std::memory_order_relaxed);
  stats_.leavesVisited.store(0, std::memory_order_relaxed);
  stats_.boidsConsidered.store(0, std::memory_order_relaxed);
  stats_.maxQueueSize.store(0, std::memory_order_relaxed);
}

// 現在の統計値を取得して同時にゼロリセットする（atomic exchange）。
// フレーム終了時に呼んで性能データを収集し、次フレーム用にリセットする。
LbvhIndex::QueryStats LbvhIndex::consumeQueryStats() const {
  QueryStats snapshot;
  snapshot.queries = stats_.queries.exchange(0, std::memory_order_relaxed);
  snapshot.nodesVisited =
    stats_.nodesVisited.exchange(0, std::memory_order_relaxed);
  snapshot.leavesVisited =
    stats_.leavesVisited.exchange(0, std::memory_order_relaxed);
  snapshot.boidsConsidered =
    stats_.boidsConsidered.exchange(0, std::memory_order_relaxed);
  snapshot.maxQueueSize =
    stats_.maxQueueSize.exchange(0, std::memory_order_relaxed);
  return snapshot;
}

LbvhIndex::QueryScratch &LbvhIndex::threadScratch() {
  // gatherNearest は多数呼び出されるため、動的確保を避ける目的でスレッドごとの
  // 作業バッファを再利用している。WASM では Web Worker ごとに thread_local が分離され、
  // レース無しに stack/best のメモリを共有できる。
  thread_local QueryScratch scratch;
  return scratch;
}
