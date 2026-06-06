#pragma once
#include <memory>

#include "nav2_teb_controller/core/timed_elastic_band.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"

namespace nav2_teb_controller {

/**
 * @brief One TEB candidate with its homotopy class and optimization cost
 */
struct TebCandidate {
  using Ptr = std::shared_ptr<TebCandidate>;

  TimedElasticBand teb;
  HSignature h_signature;
  double optimization_cost = std::numeric_limits<double>::infinity();
  bool is_feasible = false;
  bool has_diverged = false;

  /**
   * @brief Compare two candidates (for best selection)
   * Priority: feasible first, then lowest cost
   */
  bool operator<(const TebCandidate &other) const {
    if (is_feasible != other.is_feasible)
      return is_feasible > other.is_feasible;
    return optimization_cost < other.optimization_cost;
  }
};

}  // namespace nav2_teb_controller
