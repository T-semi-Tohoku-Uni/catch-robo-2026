#include "path_planning/informed_rrt_star.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace path_planning;
void require(bool condition, const char * message) {
  if (!condition) { throw std::runtime_error(message); }
}
int main() {
  PlannerOptions options;
  options.lower_bounds = {-2, -2}; options.upper_bounds = {2, 2};
  options.max_iterations = 1800;
  InformedRrtStar free(options, [](const State &) { return true; });
  auto straight = free.plan({-1, 0}, {1, 0});
  require(straight && std::abs(straight.cost-2) < 1e-12, "straight path");
  require(free.plan({0, 0}, {0, 0}).path.size() == 1, "identical endpoints");
  require(!free.plan({3, 0}, {0, 0}), "out of bounds");
  require(!free.plan({0}, {0, 0}), "dimension mismatch");
  require(!free.plan({std::numeric_limits<double>::quiet_NaN(), 0}, {0, 0}), "NaN");
  auto obstacle = [](const State & q) { return q[0]*q[0] + q[1]*q[1] > 0.36; };
  InformedRrtStar planner(options, obstacle);
  auto result = planner.plan({-1, 0}, {1, 0});
  require(result && result.cost > 2 && result.informed_samples > 0, "informed detour");
  require(result.path.front() == State({-1, 0}) && result.path.back() == State({1, 0}), "endpoints");
  double length = 0;
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    const auto & a = result.path[i-1]; const auto & b = result.path[i];
    const double dx = b[0]-a[0], dy = b[1]-a[1];
    const double t = std::max(0.0, std::min(1.0, -(a[0]*dx+a[1]*dy)/(dx*dx+dy*dy)));
    require(std::hypot(a[0]+t*dx, a[1]+t*dy) > 0.599, "edge intersects disk");
    length += std::hypot(dx, dy);
  }
  require(std::abs(length-result.cost) < 1e-9, "rewired costs agree with path");
  auto repeat = planner.plan({-1, 0}, {1, 0});
  require(result.path == repeat.path, "deterministic seed");
  auto short_options = options; short_options.max_iterations = 500;
  auto short_result = InformedRrtStar(short_options, obstacle).plan({-1, 0}, {1, 0});
  require(short_result && result.cost <= short_result.cost + 1e-12, "longer search must not worsen cost");
  InformedRrtStar blocked(options, [](const State & q) { return std::abs(q[0]) > 0.2; });
  require(!blocked.plan({-1, 0}, {1, 0}), "impenetrable wall");
  require(!planner.plan({0, 0}, {1, 0}), "blocked start");
  InformedRrtStar no_edges(options, [](const State &) { return true; },
    [](const State &, const State &) { return false; });
  require(!no_edges.plan({-1, 0}, {1, 0}), "motion callback");
  options.lower_bounds = {-2, -2, -2, -2}; options.upper_bounds = {2, 2, 2, 2};
  auto four = InformedRrtStar(options, obstacle).plan({-1, 0, 0, 0}, {1, 0, 0, 0});
  require(four && four.informed_samples > 0, "4D informed search");
  options.step_size = 0;
  bool threw = false;
  try { InformedRrtStar invalid(options, obstacle); } catch (const std::invalid_argument &) { threw = true; }
  require(threw, "invalid configuration");
  std::cout << "All planner checks passed\n";
}
