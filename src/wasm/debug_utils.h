#pragma once

#include <glm/glm.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace debug {

inline void check_invalid_value(const glm::vec3 &v,
                                std::string_view context = {}) {
  if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
    std::ostringstream oss;
    oss << "Invalid vector detected";
    if (!context.empty()) {
      oss << " in " << context;
    }
    oss << ": (" << v.x << ", " << v.y << ", " << v.z << ")";
    throw std::runtime_error(oss.str());
  }
}

inline void check_invalid_value(float value, std::string_view context = {}) {
  if (!std::isfinite(value)) {
    std::string message = "Invalid scalar detected";
    if (!context.empty()) {
      message += " in ";
      message.append(context.begin(), context.end());
    }
    message += ": " + std::to_string(value);
    throw std::runtime_error(message);
  }
}

} // namespace debug
