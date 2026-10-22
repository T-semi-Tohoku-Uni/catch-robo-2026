#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace path_planning {
using State = std::vector<double>;
// Geometric path only: a time parameterizer (e.g. TOPP) consumes this later.
using Path = std::vector<State>;
using StateValidityChecker = std::function<bool(const State &)>;
using MotionValidityChecker = std::function<bool(const State &, const State &)>;

struct PlannerOptions {
  State lower_bounds;
  State upper_bounds;
  std::size_t max_iterations{3000};
  double step_size{0.2};
  double neighbor_radius{0.6};
  double collision_resolution{0.02};
  double goal_bias{0.05};
  std::uint32_t seed{42};
};

struct PlanResult {
  Path path;
  double cost{std::numeric_limits<double>::infinity()};
  std::size_t iterations{0};
  std::size_t informed_samples{0};
  explicit operator bool() const { return !path.empty(); }
};

class InformedRrtStar {
public:
  // State checker is mandatory. Optional motion checker additionally validates
  // complete edges (e.g. continuous collision checking).
  InformedRrtStar(PlannerOptions options, StateValidityChecker state_valid,
    MotionValidityChecker motion_valid = {});
  PlanResult plan(const State & start, const State & goal) const;
  bool isStateValid(const State & state) const;
  bool isMotionValid(const State & from, const State & to) const;

private:
  PlannerOptions options_;
  StateValidityChecker state_valid_;
  MotionValidityChecker motion_valid_;
};
}  // namespace path_planning
