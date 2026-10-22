#include "path_planning/planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>

namespace path_planning {
PlannerNode::PlannerNode() : Node("path_planner") {
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names", {"theta1", "theta2", "theta3", "theta4"});
  if (joint_names_.empty() || std::set<std::string>(joint_names_.begin(),
    joint_names_.end()).size() != joint_names_.size() ||
    std::find(joint_names_.begin(), joint_names_.end(), "") != joint_names_.end()) {
    throw std::invalid_argument("joint_names must be nonempty and unique");
  }
  PlannerOptions options;
  options.lower_bounds = declare_parameter<std::vector<double>>(
    "lower_bounds", std::vector<double>(joint_names_.size(), -3.141592653589793));
  options.upper_bounds = declare_parameter<std::vector<double>>(
    "upper_bounds", std::vector<double>(joint_names_.size(), 3.141592653589793));
  const auto iterations = declare_parameter<int64_t>("max_iterations", 3000);
  const auto seed = declare_parameter<int64_t>("seed", 42);
  if (iterations <= 0 || seed < 0 || seed > UINT32_MAX ||
    options.lower_bounds.size() != joint_names_.size() ||
    options.upper_bounds.size() != joint_names_.size()) {
    throw std::invalid_argument("Invalid iterations, seed or bounds dimension");
  }
  options.max_iterations = static_cast<std::size_t>(iterations);
  options.seed = static_cast<std::uint32_t>(seed);
  options.step_size = declare_parameter<double>("step_size", 0.2);
  options.neighbor_radius = declare_parameter<double>("neighbor_radius", 0.6);
  options.collision_resolution = declare_parameter<double>("collision_resolution", 0.02);
  options.goal_bias = declare_parameter<double>("goal_bias", 0.05);
  const std::vector<std::string> expected_names{"theta1", "theta2", "theta3", "theta4"};
  if (joint_names_ != expected_names) {
    throw std::invalid_argument("Arm collision checking requires joint_names: theta1, theta2, theta3, theta4");
  }
  const auto description = declare_parameter<std::string>("robot_description", "");
  if (description.empty()) {
    throw std::invalid_argument("robot_description is required; use ros2 launch path_planning planner.launch.py");
  }
  std::vector<std::string> default_pairs;
  for (const auto & pair : ArmCollisionChecker::defaultCheckedPairs()) {
    default_pairs.push_back(pair.first + ":" + pair.second);
  }
  const auto pair_strings = declare_parameter<std::vector<std::string>>(
    "checked_collision_pairs", default_pairs);
  std::vector<LinkPair> checked_pairs;
  for (const auto & value : pair_strings) {
    const auto separator = value.find(':');
    if (separator == std::string::npos || value.find(':', separator + 1) != std::string::npos) {
      throw std::invalid_argument("checked_collision_pairs entries must be link_a:link_b");
    }
    checked_pairs.emplace_back(value.substr(0, separator), value.substr(separator + 1));
  }
  collision_checker_ = std::make_shared<ArmCollisionChecker>(description, checked_pairs);
  planner_ = std::make_unique<InformedRrtStar>(options,
    [checker = collision_checker_](const State & state) { return checker->isStateValid(state); });
  RCLCPP_INFO(get_logger(), "URDF self-collision checking enabled for configured pairs only (environment is not loaded)");
  path_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "geometric_path", rclcpp::QoS(1).transient_local());
  start_sub_ = create_subscription<sensor_msgs::msg::JointState>("start_joint_states", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      if (!decode(*msg, start_)) { RCLCPP_WARN(get_logger(), "Invalid start state; cleared"); }
    });
  goal_sub_ = create_subscription<sensor_msgs::msg::JointState>("goal_joint_states", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      if (!decode(*msg, goal_)) { RCLCPP_WARN(get_logger(), "Invalid goal state; cleared"); }
    });
  plan_service_ = create_service<std_srvs::srv::Trigger>("plan_path",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) { plan(response); });
}

bool PlannerNode::decode(const sensor_msgs::msg::JointState & message, State & state) const {
  state.clear();
  State decoded;
  if (message.name.empty()) {
    decoded = message.position;
  } else {
    if (message.name.size() != message.position.size() ||
      std::set<std::string>(message.name.begin(), message.name.end()).size() != message.name.size()) {
      return false;
    }
    for (const auto & name : joint_names_) {
      auto it = std::find(message.name.begin(), message.name.end(), name);
      if (it == message.name.end()) { return false; }
      decoded.push_back(message.position[std::distance(message.name.begin(), it)]);
    }
  }
  const auto collision = collision_checker_->check(decoded);
  if (!collision.valid) {
    RCLCPP_WARN(get_logger(), "%s", collision.reason.c_str());
    return false;
  }
  if (!planner_->isStateValid(decoded)) { return false; }
  state = std::move(decoded);
  return true;
}

void PlannerNode::plan(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
  trajectory_msgs::msg::JointTrajectory message;
  message.header.stamp = now();
  message.joint_names = joint_names_;
  response->success = false;
  if (start_.empty() || goal_.empty()) {
    response->message = "Publish valid start_joint_states and goal_joint_states first.";
  } else {
    const auto result = planner_->plan(start_, goal_);
    response->success = static_cast<bool>(result);
    response->message = result ? "Geometric path generated; cost=" + std::to_string(result.cost) :
      "No path found within iteration budget.";
    for (const auto & state : result.path) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = state;
      // Intentionally untimed: never send this topic directly to a controller.
      message.points.push_back(std::move(point));
    }
  }
  // Also clear any latched old path on failure.
  path_pub_->publish(message);
}
}  // namespace path_planning
