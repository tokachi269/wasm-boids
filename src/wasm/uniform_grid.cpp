#include "uniform_grid.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <glm/glm.hpp>

namespace {
// ハッシュ用の大きめの素数。セル座標の衝突を減らす目的で利用する。
constexpr std::size_t kPrimeX = 73856093u;
constexpr std::size_t kPrimeY = 19349663u;
constexpr std::size_t kPrimeZ = 83492791u;
constexpr float kMinCellSize = 1e-2f;

constexpr int kCellSampleLimit = 256;   // 1セルあたり最大走査数

struct Candidate {
  float distanceSq = 0.0f;
  int index = -1;
};

struct CandidateScratch {
  std::vector<Candidate> heap;
  std::vector<int> samples;
};

thread_local CandidateScratch gCandidateScratch;

inline std::uint32_t hashCombine(std::uint32_t seed, int value) {
  constexpr std::uint32_t kMul = 0x9e3779b9u;
  std::uint32_t v = static_cast<std::uint32_t>(value);
  seed ^= v + kMul + (seed << 6) + (seed >> 2);
  return seed;
}

inline std::uint32_t cellSeed(std::uint32_t baseSeed, int x, int y, int z) {
  std::uint32_t seed = hashCombine(baseSeed, x);
  seed = hashCombine(seed, y);
  seed = hashCombine(seed, z);
  return seed | 1u; // LCG の係数がゼロにならないよう最低1を確保
}
} // namespace

namespace {
inline std::uint32_t lcgStep(std::uint32_t &state) {
  state = state * 1664525u + 1013904223u;
  return state;
}
} // namespace

UniformGridIndex::UniformGridIndex(float cellSize) { setCellSize(cellSize); }

void UniformGridIndex::setCellSize(float cellSize) {
  cellSize_ = std::max(cellSize, kMinCellSize);
  invCellSize_ = 1.0f / cellSize_;
}

void UniformGridIndex::setSamplingSeed(std::uint32_t seed) {
  samplingSeed_ = seed;
}

void UniformGridIndex::clear() {
  buffers_ = nullptr;
  cells_.clear();
}

UniformGridIndex::CellKey UniformGridIndex::cellFor(const glm::vec3 &position) const {
  return CellKey{static_cast<int>(std::floor(position.x * invCellSize_)),
                 static_cast<int>(std::floor(position.y * invCellSize_)),
                 static_cast<int>(std::floor(position.z * invCellSize_))};
}

std::size_t UniformGridIndex::KeyHash::operator()(const CellKey &key) const noexcept {
  const std::size_t x = static_cast<std::size_t>(static_cast<std::uint64_t>(key.x) * kPrimeX);
  const std::size_t y = static_cast<std::size_t>(static_cast<std::uint64_t>(key.y) * kPrimeY);
  const std::size_t z = static_cast<std::size_t>(static_cast<std::uint64_t>(key.z) * kPrimeZ);
  return x ^ y ^ z;
}

bool UniformGridIndex::KeyEq::operator()(const CellKey &lhs, const CellKey &rhs) const noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

void UniformGridIndex::build(const SoABuffers &buffers) {
  buffers_ = &buffers;
  cells_.clear();

  const std::size_t count = buffers.positions.size();
  if (count == 0) {
    return;
  }

  cells_.reserve(static_cast<std::size_t>(count * 1.2f));
  for (std::size_t i = 0; i < count; ++i) {
    const glm::vec3 &pos = buffers.positions[i];
    const CellKey key = cellFor(pos);
    auto &cell = cells_[key];
    cell.indices.push_back(static_cast<int>(i));
  }
}

int UniformGridIndex::gatherNearest(const glm::vec3 &center, int maxCount,
                                    float maxRadius, int *outIndices,
                                    float *outDistancesSq) const {
  if (maxCount <= 0 || !outIndices || !outDistancesSq || !buffers_) {
    return 0;
  }

  const auto &positions = buffers_->positions;
  if (positions.empty()) {
    return 0;
  }

  const float baseRadius = (maxRadius > 0.0f) ? maxRadius : cellSize_;
  const float baseRadiusSq = baseRadius * baseRadius;
  const int maxLayer = std::max(1, static_cast<int>(std::ceil(baseRadius * invCellSize_)));

  auto &scratch = gCandidateScratch;
  auto &heap = scratch.heap;
  heap.clear();
  heap.reserve(static_cast<std::size_t>(maxCount));

  const CellKey centerCell = cellFor(center);

  for (int dz = -maxLayer; dz <= maxLayer; ++dz) {
    for (int dy = -maxLayer; dy <= maxLayer; ++dy) {
      for (int dx = -maxLayer; dx <= maxLayer; ++dx) {
        const CellKey key{centerCell.x + dx, centerCell.y + dy, centerCell.z + dz};
        const auto it = cells_.find(key);
        if (it == cells_.end()) {
          continue;
        }
        const auto &indices = it->second.indices;
        const int count = static_cast<int>(indices.size());
        const bool needsSampling = count > kCellSampleLimit;

        if (!needsSampling) {
          for (int boidIndex : indices) {
            const glm::vec3 &pos =
                positions[static_cast<std::size_t>(boidIndex)];
            const glm::vec3 diff = pos - center;
            const float distSq = glm::dot(diff, diff);
            if (distSq <= baseRadiusSq) {
              if (heap.size() < static_cast<std::size_t>(maxCount)) {
                heap.push_back(Candidate{distSq, boidIndex});
                std::push_heap(heap.begin(), heap.end(),
                               [](const Candidate &lhs,
                                  const Candidate &rhs) {
                                 return lhs.distanceSq < rhs.distanceSq;
                               });
              } else if (!heap.empty() &&
                         distSq < heap.front().distanceSq) {
                std::pop_heap(heap.begin(), heap.end(),
                              [](const Candidate &lhs,
                                 const Candidate &rhs) {
                                return lhs.distanceSq < rhs.distanceSq;
                              });
                heap.back() = Candidate{distSq, boidIndex};
                std::push_heap(heap.begin(), heap.end(),
                               [](const Candidate &lhs,
                                  const Candidate &rhs) {
                                 return lhs.distanceSq < rhs.distanceSq;
                               });
              }
            }
          }
          continue;
        }

        // Reservoir sampling でセル上限数のみ走査（公平性を保ちつつ負荷を制限）
        auto &samples = scratch.samples;
        samples.clear();
        samples.reserve(kCellSampleLimit);

        const int sampleCount = std::min(kCellSampleLimit, count);
        samples.resize(sampleCount);

    std::uint32_t state =
      cellSeed(samplingSeed_, centerCell.x + dx, centerCell.y + dy,
           centerCell.z + dz);

        for (int i = 0; i < sampleCount; ++i) {
          samples[static_cast<std::size_t>(i)] = indices[i];
        }
        for (int i = sampleCount; i < count; ++i) {
          const std::uint32_t r = lcgStep(state) % (i + 1);
          if (static_cast<int>(r) < sampleCount) {
            samples[static_cast<std::size_t>(r)] = indices[i];
          }
        }

        for (int boidIndex : samples) {
          const glm::vec3 &pos =
              positions[static_cast<std::size_t>(boidIndex)];
          const glm::vec3 diff = pos - center;
          const float distSq = glm::dot(diff, diff);
          if (distSq > baseRadiusSq) {
            continue;
          }

          if (heap.size() < static_cast<std::size_t>(maxCount)) {
            heap.push_back(Candidate{distSq, boidIndex});
            std::push_heap(heap.begin(), heap.end(),
                           [](const Candidate &lhs, const Candidate &rhs) {
                             return lhs.distanceSq < rhs.distanceSq;
                           });
            continue;
          }

          if (!heap.empty() && distSq < heap.front().distanceSq) {
            std::pop_heap(heap.begin(), heap.end(),
                          [](const Candidate &lhs, const Candidate &rhs) {
                            return lhs.distanceSq < rhs.distanceSq;
                          });
            heap.back() = Candidate{distSq, boidIndex};
            std::push_heap(heap.begin(), heap.end(),
                           [](const Candidate &lhs, const Candidate &rhs) {
                             return lhs.distanceSq < rhs.distanceSq;
                           });
          }
        }
      }
    }
  }

  const int resultCount = static_cast<int>(heap.size());
  if (resultCount <= 0) {
    return 0;
  }

  std::sort(heap.begin(), heap.end(),
            [](const Candidate &a, const Candidate &b) {
              return a.distanceSq < b.distanceSq;
            });

  for (int i = 0; i < resultCount; ++i) {
    outDistancesSq[i] = heap[static_cast<std::size_t>(i)].distanceSq;
    outIndices[i] = heap[static_cast<std::size_t>(i)].index;
  }
  return resultCount;
}

bool UniformGridIndex::getLeafMembers(int boidIndex, const int *&outIndices,
                                      int &count) const {
  outIndices = nullptr;
  count = 0;
  if (!buffers_ || boidIndex < 0) {
    return false;
  }

  const auto &positions = buffers_->positions;
  if (boidIndex >= static_cast<int>(positions.size())) {
    return false;
  }

  const CellKey key = cellFor(positions[static_cast<std::size_t>(boidIndex)]);
  const auto it = cells_.find(key);
  if (it == cells_.end() || it->second.indices.empty()) {
    return false;
  }

  outIndices = it->second.indices.data();
  count = static_cast<int>(it->second.indices.size());
  return true;
}

void UniformGridIndex::forEachLeaf(const LeafVisitor &visitor) const {
  if (!visitor) {
    return;
  }
  for (const auto &kv : cells_) {
    const auto &indices = kv.second.indices;
    if (indices.empty()) {
      continue;
    }
    visitor(SpatialLeaf{indices.data(), indices.size(), nullptr});
  }
}

void UniformGridIndex::forEachLeafIntersectingSphere(
    const glm::vec3 &center, float radius, const LeafVisitor &visitor) const {
  if (!visitor || radius <= 0.0f) {
    return;
  }

  const int minX = static_cast<int>(std::floor((center.x - radius) * invCellSize_));
  const int maxX = static_cast<int>(std::floor((center.x + radius) * invCellSize_));
  const int minY = static_cast<int>(std::floor((center.y - radius) * invCellSize_));
  const int maxY = static_cast<int>(std::floor((center.y + radius) * invCellSize_));
  const int minZ = static_cast<int>(std::floor((center.z - radius) * invCellSize_));
  const int maxZ = static_cast<int>(std::floor((center.z + radius) * invCellSize_));

  const float radiusSq = radius * radius;

  for (int z = minZ; z <= maxZ; ++z) {
    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        const CellKey key{x, y, z};
        const auto it = cells_.find(key);
        if (it == cells_.end()) {
          continue;
        }
        const auto &indices = it->second.indices;
        if (indices.empty()) {
          continue;
        }
        // AABB と球の交差判定を簡易的に行う。
        const glm::vec3 cellMin(static_cast<float>(x) * cellSize_,
                                 static_cast<float>(y) * cellSize_,
                                 static_cast<float>(z) * cellSize_);
        const glm::vec3 cellMax = cellMin + glm::vec3(cellSize_);
        const glm::vec3 clamped = glm::clamp(center, cellMin, cellMax);
        const glm::vec3 diff = clamped - center;
        const float distSq = glm::dot(diff, diff);
        if (distSq > radiusSq) {
          continue;
        }
        visitor(SpatialLeaf{indices.data(), indices.size(), nullptr});
      }
    }
  }
}
