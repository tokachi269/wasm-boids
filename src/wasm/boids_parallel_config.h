#pragma once

#include <atomic>
#include <cstddef>

inline std::atomic<std::size_t>& boidsMaxTasksOverride() {
  static std::atomic<std::size_t> value{0};
  return value;
}

inline void setBoidsMaxTasksOverride(std::size_t value) {
  boidsMaxTasksOverride().store(value, std::memory_order_relaxed);
}
