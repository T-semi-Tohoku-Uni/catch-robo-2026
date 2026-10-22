#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "path_planning/informed_rrt_star.hpp"
#include "path_planning/arm_collision_checker.hpp"

namespace path_planning {
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();
private:
  bool decode(const sensor_msgs::msg::JointState & message, State & state) const;
  void plan(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response);
  std::vector<std::string> joint_names_;
  std::unique_ptr<InformedRrtStar> planner_;
  State start_, goal_;
  std::shared_ptr<ArmCollisionChecker> collision_checker_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr start_sub_, goal_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
};
}  // namespace path_planning
