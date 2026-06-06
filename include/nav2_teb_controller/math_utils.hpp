#pragma once

#include <cmath>

inline double average_angle(double angle1, double angle2) {
  // Convert to unit vectors and average them
  double x = std::cos(angle1) + std::cos(angle2);
  double y = std::sin(angle1) + std::sin(angle2);

  // Handle the case where vectors cancel out
  if (std::abs(x) < 1e-10 && std::abs(y) < 1e-10) {
    return 0.0;
  }

  return std::atan2(y, x);
}

/**
 * @brief Calculate a fast approximation of a sigmoid function
 * @details The following function is implemented: \f$ x / (1 + |x|) \f$
 * @param x the argument of the function
 */
[[nodiscard]] constexpr double fast_sigmoid(double x) noexcept {
  return x / (1.0 + std::abs(x));
}
