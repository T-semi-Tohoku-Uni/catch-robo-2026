#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "path_planning/arm_collision_checker.hpp"

namespace path_planning {
class PathPreviewNode : public rclcpp::Node {
public:
  PathPreviewNode();
private:
  void receive(const trajectory_msgs::msg::JointTrajectory & message);
  void clear();
  std::unique_ptr<ArmCollisionChecker> model_;
  double resolution_{0.02};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr path_sub_;
};
}  // namespace path_planning
