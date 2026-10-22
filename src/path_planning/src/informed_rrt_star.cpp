#include "path_planning/informed_rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

namespace path_planning {
namespace {
double distance(const State & a, const State & b) {
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) { sum += (a[i]-b[i])*(a[i]-b[i]); }
  return std::sqrt(sum);
}
State interpolate(const State & a, const State & b, double t) {
  State q(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) { q[i] = a[i] + t*(b[i]-a[i]); }
  return q;
}
struct Vertex { State q; std::size_t parent; double cost; };
}  // namespace

InformedRrtStar::InformedRrtStar(PlannerOptions options,
  StateValidityChecker state_valid, MotionValidityChecker motion_valid)
: options_(std::move(options)), state_valid_(std::move(state_valid)),
  motion_valid_(std::move(motion_valid)) {
  const auto & o = options_;
  if (o.lower_bounds.empty() || o.lower_bounds.size() != o.upper_bounds.size() ||
    !state_valid_ || o.max_iterations == 0 || !std::isfinite(o.step_size) ||
    o.step_size <= 0 || !std::isfinite(o.neighbor_radius) || o.neighbor_radius <= 0 ||
    !std::isfinite(o.collision_resolution) || o.collision_resolution <= 0 ||
    !std::isfinite(o.goal_bias) || o.goal_bias < 0 || o.goal_bias >= 1) {
    throw std::invalid_argument("Invalid planner options or missing state checker");
  }
  for (std::size_t i = 0; i < o.lower_bounds.size(); ++i) {
    if (!std::isfinite(o.lower_bounds[i]) || !std::isfinite(o.upper_bounds[i]) ||
      o.lower_bounds[i] >= o.upper_bounds[i]) {
      throw std::invalid_argument("Bounds must be finite and strictly increasing");
    }
  }
}

bool InformedRrtStar::isStateValid(const State & q) const {
  if (q.size() != options_.lower_bounds.size()) { return false; }
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i]) || q[i] < options_.lower_bounds[i] ||
      q[i] > options_.upper_bounds[i]) { return false; }
  }
  return state_valid_(q);
}

bool InformedRrtStar::isMotionValid(const State & a, const State & b) const {
  if (!isStateValid(a) || !isStateValid(b)) { return false; }
  const auto steps = static_cast<std::size_t>(
    std::max(1.0, std::ceil(distance(a, b)/options_.collision_resolution)));
  for (std::size_t i = 1; i < steps; ++i) {
    if (!isStateValid(interpolate(a, b, static_cast<double>(i)/steps))) { return false; }
  }
  return !motion_valid_ || motion_valid_(a, b);
}

PlanResult InformedRrtStar::plan(const State & start, const State & goal) const {
  PlanResult result;
  if (!isStateValid(start) || !isStateValid(goal)) { return result; }
  const double minimum_cost = distance(start, goal);
  if (isMotionValid(start, goal)) {
    result.path = minimum_cost == 0 ? Path{start} : Path{start, goal};
    result.cost = minimum_cost;
    return result;
  }
  const auto dim = start.size();
  std::mt19937 rng(options_.seed);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::normal_distribution<double> normal(0.0, 1.0);
  // Householder reflection maps the first unit axis to the start-goal axis.
  State reflection(dim);
  for (std::size_t i = 0; i < dim; ++i) {
    reflection[i] = (i == 0 ? 1.0 : 0.0) - (goal[i]-start[i])/minimum_cost;
  }
  double reflection_norm = 0;
  for (double v : reflection) { reflection_norm += v*v; }
  std::vector<Vertex> tree{{start, 0, 0.0}};
  std::vector<std::size_t> goal_parents;
  std::size_t best_parent = 0;
  for (std::size_t iteration = 0; iteration < options_.max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    if (result.cost <= minimum_cost + 1e-12) { break; }
    State sample(dim);
    if (uniform(rng) < options_.goal_bias) {
      sample = goal;
    } else if (std::isfinite(result.cost)) {
      double norm = 0;
      for (double & v : sample) { v = normal(rng); norm += v*v; }
      if (norm == 0) { continue; }
      const double radius = std::pow(uniform(rng), 1.0/dim)/std::sqrt(norm);
      const double minor = std::sqrt(std::max(0.0,
        (result.cost-minimum_cost)*(result.cost+minimum_cost)))/2;
      for (std::size_t i = 0; i < dim; ++i) {
        sample[i] *= radius*(i == 0 ? result.cost/2 : minor);
      }
      double dot = 0;
      for (std::size_t i = 0; i < dim; ++i) { dot += reflection[i]*sample[i]; }
      for (std::size_t i = 0; i < dim; ++i) {
        if (reflection_norm > 1e-24) { sample[i] -= 2*reflection[i]*dot/reflection_norm; }
        sample[i] += (start[i]+goal[i])/2;
      }
      ++result.informed_samples;
    } else {
      for (std::size_t i = 0; i < dim; ++i) {
        sample[i] = options_.lower_bounds[i] + uniform(rng)*
          (options_.upper_bounds[i]-options_.lower_bounds[i]);
      }
    }
    if (!isStateValid(sample)) { continue; }
    std::size_t nearest = 0;
    double nearest_distance = distance(tree[0].q, sample);
    for (std::size_t i = 1; i < tree.size(); ++i) {
      const double d = distance(tree[i].q, sample);
      if (d < nearest_distance) { nearest = i; nearest_distance = d; }
    }
    if (nearest_distance < 1e-12) { continue; }
    State q = interpolate(tree[nearest].q, sample,
      std::min(1.0, options_.step_size/nearest_distance));
    if (!isMotionValid(tree[nearest].q, q)) { continue; }
    std::size_t parent = nearest;
    double cost = tree[parent].cost + distance(tree[parent].q, q);
    std::vector<std::size_t> neighbors;
    // Fixed positive radius: simple exhaustive neighborhood, no spatial index.
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d = distance(tree[i].q, q);
      if (d > options_.neighbor_radius) { continue; }
      neighbors.push_back(i);
      if (tree[i].cost+d < cost && isMotionValid(tree[i].q, q)) {
        parent = i; cost = tree[i].cost+d;
      }
    }
    const auto added = tree.size();
    tree.push_back({q, parent, cost});
    for (auto i : neighbors) {
      const double new_cost = cost + distance(q, tree[i].q);
      if (i == 0 || new_cost + 1e-12 >= tree[i].cost) { continue; }
      // Explicitly prevent cycles, including under floating point roundoff.
      bool ancestor = false;
      for (auto p = parent;; p = tree[p].parent) {
        if (p == i) { ancestor = true; break; }
        if (p == 0) { break; }
      }
      if (ancestor || !isMotionValid(q, tree[i].q)) { continue; }
      tree[i].parent = added;
      tree[i].cost = new_cost;
      // Recompute descendants after rewiring; insertion order is not topological.
      std::vector<std::size_t> pending{i};
      while (!pending.empty()) {
        auto p = pending.back(); pending.pop_back();
        for (std::size_t j = 1; j < tree.size(); ++j) {
          if (tree[j].parent == p) {
            tree[j].cost = tree[p].cost + distance(tree[p].q, tree[j].q);
            pending.push_back(j);
          }
        }
      }
    }
    if (distance(q, goal) <= options_.step_size && isMotionValid(q, goal)) {
      goal_parents.push_back(added);
    }
    for (auto i : goal_parents) {
      const double candidate = tree[i].cost + distance(tree[i].q, goal);
      if (candidate < result.cost) { result.cost = candidate; best_parent = i; }
    }
  }
  if (std::isfinite(result.cost)) {
    result.path.push_back(goal);
    for (auto p = best_parent;; p = tree[p].parent) {
      if (distance(result.path.back(), tree[p].q) > 0) { result.path.push_back(tree[p].q); }
      if (p == 0) { break; }
    }
    std::reverse(result.path.begin(), result.path.end());
  }
  return result;
}
}  // namespace path_planning
