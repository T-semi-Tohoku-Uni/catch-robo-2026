#include "path_planner/rrt_connect.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace path_planner
{
namespace
{
enum class StepStatus {kTrapped, kAdvanced, kReached};

struct Tree
{
  std::vector<State> nodes;
  std::vector<int> parents;
};

Path reconstructChain(const Tree & tree, int index)
{
  Path out;
  for (int cur = index; cur != -1; cur = tree.parents[static_cast<std::size_t>(cur)]) {
    out.push_back(tree.nodes[static_cast<std::size_t>(cur)]);
  }
  std::reverse(out.begin(), out.end());
  return out;
}

RobotCollisionModel makeLegacyCollisionModel(const ArmLinkDimensions & dim)
{
  RobotCollisionModel model;
  RobotShape upper_arm;
  upper_arm.name = "upper_arm";
  upper_arm.link = "upper_arm_link";
  upper_arm.link_index = 2;
  upper_arm.shape.type = ShapeType::kBox;
  upper_arm.shape.half_extents = {
    dim.upper_arm_size[0] * 0.5, dim.upper_arm_size[1] * 0.5, dim.upper_arm_size[2] * 0.5};
  upper_arm.shape.pose = makeTransform({0.0, 0.0, dim.upper_arm_length * 0.5}, {0.0, 0.0, 0.0});

  RobotShape flange;
  flange.name = "flange";
  flange.link = "flange_link";
  flange.link_index = 5;
  flange.shape.type = ShapeType::kBox;
  flange.shape.half_extents = {
    dim.hand_size[0] * 0.5, dim.hand_size[1] * 0.5, dim.hand_size[2] * 0.5};
  flange.shape.pose = makeTransform({0.0, 0.0, -dim.hand_size[2] * 0.5}, {0.0, 0.0, kPi * 0.5});

  model.shapes.push_back(upper_arm);
  model.shapes.push_back(flange);
  model.self_collision_pairs.emplace_back(0, 1);
  return model;
}
}  // namespace

RrtConnect::RrtConnect(
  RrtConnectConfig config,
  std::shared_ptr<const StateValidator> validator)
: config_(std::move(config)), validator_(std::move(validator))
{
  if (!validator_) {
    throw std::invalid_argument("RrtConnect requires a non-null StateValidator");
  }
  if (config_.bounds.lower.size() != kDof || config_.bounds.upper.size() != kDof) {
    throw std::invalid_argument("RrtConnect bounds must have 4 elements");
  }
  if(config_.active_axes!=3 && config_.active_axes!=4) throw std::invalid_argument("active_axes must be 3 or 4");
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!std::isfinite(config_.bounds.lower[i]) || !std::isfinite(config_.bounds.upper[i]) ||
      (i<config_.active_axes ? config_.bounds.lower[i]>=config_.bounds.upper[i] :
       config_.bounds.lower[i]!=0.0 || config_.bounds.upper[i]!=0.0))
    {
      throw std::invalid_argument("RrtConnect bounds require lower < upper");
    }
  }
  if (config_.distance_weights.size() != kDof) {
    throw std::invalid_argument("distance_weights must have 4 elements");
  }
  if (!std::isfinite(config_.distance_weights[0]) || config_.distance_weights[0] < 1.0) {
    throw std::invalid_argument("distance_weights[0] must be >= 1.0");
  }
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!std::isfinite(config_.distance_weights[i]) || (i<config_.active_axes ? config_.distance_weights[i]<=0.0 : config_.distance_weights[i]!=0.0)) {
      throw std::invalid_argument("distance_weights must be positive and finite");
    }
  }
  if (!std::isfinite(config_.step_size) || config_.step_size <= 0.0) {
    throw std::invalid_argument("step_size must be > 0");
  }
  if (!std::isfinite(config_.goal_bias) || config_.goal_bias < 0.0 || config_.goal_bias > 1.0) {
    throw std::invalid_argument("goal_bias must be in [0, 1]");
  }
  if (config_.max_iterations == 0) {
    throw std::invalid_argument("max_iterations must be >= 1");
  }
}

double RrtConnect::weightedDistance(const State & a, const State & b) const
{
  double sum = 0.0;
  for (std::size_t i = 0; i < kDof; ++i) {
    const double d = config_.distance_weights[i] * (a[i] - b[i]);
    sum += d * d;
  }
  return std::sqrt(sum);
}

PlanResult RrtConnect::plan(
  const State & start, const State & goal,
  std::uint32_t seed) const
{
  PlanResult result;
  if (!isFiniteState(start) || !isFiniteState(goal)) {
    result.failure = FailureCode::kInvalidInput;
    result.message = "start or goal has non-finite or non-4D state";
    return result;
  }
  if (!config_.bounds.contains(start) || !config_.bounds.contains(goal)) {
    result.failure = FailureCode::kJointLimitViolation;
    result.message = "start or goal violates joint bounds";
    return result;
  }
  if (!validator_->isStateValid(start)) {
    result.failure = FailureCode::kCollisionDetected;
    result.message = "start state is in collision or invalid";
    return result;
  }
  if (!validator_->isStateValid(goal)) {
    result.failure = FailureCode::kCollisionDetected;
    result.message = "goal state is in collision or invalid";
    return result;
  }

  // 同一始終点または直接接続可能なら即座に返す。
  if (weightedDistance(start, goal) <= 1e-12) {
    result.path = {start, goal};
    result.iterations = 0;
    return result;
  }

  const auto nearest_index = [&](const Tree & tree, const State & target) {
      int best = 0;
      double best_dist = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const double d = weightedDistance(tree.nodes[i], target);
        if (d < best_dist) {
          best_dist = d;
          best = static_cast<int>(i);
        }
      }
      return best;
    };

  const auto extend_from = [&](
    Tree & tree, int from_idx, const State & target, int & new_idx) -> StepStatus {
      const State & from = tree.nodes[static_cast<std::size_t>(from_idx)];
      const double dist = weightedDistance(from, target);
      if (dist <= 1e-12) {
        new_idx = from_idx;
        return StepStatus::kReached;
      }
      State next(kDof);
      bool reached = false;
      if (dist <= config_.step_size) {
        next = target;
        reached = true;
      } else {
        const double ratio = config_.step_size / dist;
        for (std::size_t i = 0; i < kDof; ++i) {
          next[i] = from[i] + ratio * (target[i] - from[i]);
        }
      }
      if (!config_.bounds.contains(next) || !validator_->isMotionValid(from, next)) {
        return StepStatus::kTrapped;
      }
      new_idx = static_cast<int>(tree.nodes.size());
      tree.nodes.push_back(std::move(next));
      tree.parents.push_back(from_idx);
      return reached ? StepStatus::kReached : StepStatus::kAdvanced;
    };

  const auto extend = [&](Tree & tree, const State & target, int & new_idx) -> StepStatus {
      const int near_idx = nearest_index(tree, target);
      return extend_from(tree, near_idx, target, new_idx);
    };

  // EXTEND 成功後は同じ目標へ TRAPPED または REACHED まで反復して伸ばす（仕様書 第7.2節）。
  const auto connect = [&](Tree & tree, const State & target, int & last_idx) -> StepStatus {
      int current_idx = nearest_index(tree, target);
      StepStatus status = StepStatus::kAdvanced;
      while (status == StepStatus::kAdvanced) {
        int next_idx = current_idx;
        status = extend_from(tree, current_idx, target, next_idx);
        if (status != StepStatus::kTrapped) {
          current_idx = next_idx;
        }
      }
      last_idx = current_idx;
      return status;
    };

  Tree tree_a{{start}, {-1}};
  Tree tree_b{{goal}, {-1}};
  bool a_is_start = true;

  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
  std::array<std::uniform_real_distribution<double>, kDof> joint_dists{
    std::uniform_real_distribution<double>(config_.bounds.lower[0], config_.bounds.upper[0]),
    std::uniform_real_distribution<double>(config_.bounds.lower[1], config_.bounds.upper[1]),
    std::uniform_real_distribution<double>(config_.bounds.lower[2], config_.bounds.upper[2]),
    std::uniform_real_distribution<double>(config_.bounds.lower[3], config_.bounds.upper[3])};

  for (std::size_t iter = 0; iter < config_.max_iterations; ++iter) {
    result.iterations = iter + 1;
    State q_rand(kDof);
    if (unit_dist(rng) < config_.goal_bias) {
      q_rand = tree_b.nodes.front();
    } else {
      for (std::size_t i = 0; i < kDof; ++i) {
        q_rand[i] = joint_dists[i](rng);
      }
    }

    int new_a_idx = -1;
    if (extend(tree_a, q_rand, new_a_idx) != StepStatus::kTrapped) {
      const State q_new = tree_a.nodes[static_cast<std::size_t>(new_a_idx)];
      int new_b_idx = -1;
      if (connect(tree_b, q_new, new_b_idx) == StepStatus::kReached) {
        const Tree & start_tree = a_is_start ? tree_a : tree_b;
        const Tree & goal_tree = a_is_start ? tree_b : tree_a;
        const int start_idx = a_is_start ? new_a_idx : new_b_idx;
        const int goal_idx = a_is_start ? new_b_idx : new_a_idx;

        Path from_start = reconstructChain(start_tree, start_idx);
        Path from_goal = reconstructChain(goal_tree, goal_idx);
        std::reverse(from_goal.begin(), from_goal.end());
        // 接続点の重複を除いて連結する
        for (std::size_t k = 1; k < from_goal.size(); ++k) {
          from_start.push_back(from_goal[k]);
        }
        result.path = std::move(from_start);
        return result;
      }
    }

    std::swap(tree_a, tree_b);
    a_is_start = !a_is_start;
  }

  result.failure = FailureCode::kPlanningFailed;
  result.message = "RRT-Connect failed to connect trees within max_iterations";
  return result;
}

// ============================================================================
// 後方互換用 RRT_connect
// ============================================================================

bool RRT_connect::isStateCollision(const std::array<double, 4> & state)
{
  const State q(state.begin(), state.end());
  auto scene = std::make_shared<CollisionScene>();
  CollisionChecker checker(
    defaultKinematicChain(), makeLegacyCollisionModel(options_.arm_dimensions), scene, 0.0);
  return checker.isColliding(q);
}

bool RRT_connect::isObstacleDetection(
  const std::array<double, 4> & start,
  const std::array<double, 4> & end)
{
  const State a(start.begin(), start.end());
  const State b(end.begin(), end.end());
  auto scene = std::make_shared<CollisionScene>();
  auto checker = std::make_shared<CollisionChecker>(
    defaultKinematicChain(), makeLegacyCollisionModel(options_.arm_dimensions), scene, 0.0);
  JointBounds bounds{{-kPi, -kPi, -kPi, -kPi}, {kPi, kPi, kPi, kPi}};
  ConfigurationValidator validator(bounds, checker, std::max(1e-6, options_.collision_resolution));
  return !validator.isMotionValid(a, b);
}

bool RRT_connect::isPathPlanning(
  const std::array<double, 4> & start,
  const std::array<double, 4> & end,
  std::vector<std::array<double, 4>> & path)
{
  path.clear();
  auto scene = std::make_shared<CollisionScene>();
  auto checker = std::make_shared<CollisionChecker>(
    defaultKinematicChain(), makeLegacyCollisionModel(options_.arm_dimensions), scene, 0.0);
  RrtConnectConfig cfg;
  cfg.bounds = {{-kPi, -kPi, -kPi, -kPi}, {kPi, kPi, kPi, kPi}};
  cfg.distance_weights = {1.0, 1.0, 1.0, 1.0};
  cfg.step_size = std::max(1e-4, options_.step_size);
  cfg.max_iterations = std::max<std::size_t>(1, options_.max_iterations);
  auto validator = std::make_shared<ConfigurationValidator>(
    cfg.bounds, checker, std::max(1e-6, options_.collision_resolution));
  RrtConnect planner(cfg, validator);
  const State s(start.begin(), start.end());
  const State g(end.begin(), end.end());
  const PlanResult res = planner.plan(s, g, 42);
  if (!res) {
    return false;
  }
  path.reserve(res.path.size());
  for (const auto & p : res.path) {
    path.push_back({p[0], p[1], p[2], p[3]});
  }
  return true;
}

}  // namespace path_planner
