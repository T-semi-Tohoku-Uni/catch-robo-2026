#include "path_planning/path_preview_node.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace path_planning {
namespace {
const std::vector<std::string> names{"theta1", "theta2", "theta3", "theta4"};
double distance(const State & a, const State & b) {
  double sum = 0;
  for (std::size_t i = 0; i < a.size(); ++i) { sum += (a[i]-b[i])*(a[i]-b[i]); }
  return std::sqrt(sum);
}
State interpolate(const State & a, const State & b, double t) {
  State q(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) { q[i] = a[i] + t*(b[i]-a[i]); }
  return q;
}
}

PathPreviewNode::PathPreviewNode() : Node("path_preview") {
  resolution_ = declare_parameter<double>("preview_resolution", 0.02);
  if (!std::isfinite(resolution_) || resolution_ <= 0) {
    throw std::invalid_argument("Preview resolution must be finite and positive");
  }
  model_ = std::make_unique<ArmCollisionChecker>(
    declare_parameter<std::string>("robot_description", ""));
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "path_marker", rclcpp::QoS(1).transient_local());
  path_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "geometric_path", rclcpp::QoS(1).transient_local(),
    [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) { receive(*msg); });
}

void PathPreviewNode::clear() {
  // A failed or malformed plan must not leave the previous path visible.
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now(); marker.ns = "geometric_path"; marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker_pub_->publish(marker);
}

void PathPreviewNode::receive(const trajectory_msgs::msg::JointTrajectory & message) {
  clear();
  if (message.points.empty()) { return; }
  try {
    if (message.joint_names.size() != 4 ||
      std::set<std::string>(message.joint_names.begin(), message.joint_names.end()).size() != 4) {
      throw std::invalid_argument("Preview needs the four named absolute joints");
    }
    std::vector<std::size_t> indices;
    for (const auto & name : names) {
      auto it = std::find(message.joint_names.begin(), message.joint_names.end(), name);
      if (it == message.joint_names.end()) { throw std::invalid_argument("Missing joint " + name); }
      indices.push_back(std::distance(message.joint_names.begin(), it));
    }
    Path path;
    std::vector<double> lengths;
    for (const auto & point : message.points) {
      if (point.positions.size() != 4) { throw std::invalid_argument("Wrong position dimension"); }
      State q;
      for (auto index : indices) { q.push_back(point.positions[index]); }
      model_->linkPosition(q, "flange_link");
      const double length = path.empty() ? 0 : distance(path.back(), q);
      if (!std::isfinite(length)) { throw std::invalid_argument("Path length overflow"); }
      if (!path.empty() && length < 1e-12) { continue; }
      lengths.push_back(path.empty() ? 0 : lengths.back() + length);
      path.push_back(std::move(q));
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now(); marker.ns = "geometric_path"; marker.id = 0;
    marker.type = path.size() == 1 ? visualization_msgs::msg::Marker::SPHERE_LIST :
      visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
    marker.color.g = 1; marker.color.b = 0.3; marker.color.a = 1;
    auto append_point = [&](const State & q) {
        const auto p = model_->linkPosition(q, "flange_link");
        geometry_msgs::msg::Point point;
        point.x = p[0]; point.y = p[1]; point.z = p[2];
        marker.points.push_back(point);
      };
    append_point(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double count = std::ceil((lengths[i]-lengths[i-1])/resolution_);
      if (count > 100000 || marker.points.size() + count > 100000) {
        throw std::invalid_argument("Preview path exceeds 100000 display samples");
      }
      const auto steps = static_cast<std::size_t>(std::max(1.0, count));
      for (std::size_t k = 1; k <= steps; ++k) {
        append_point(interpolate(path[i-1], path[i], static_cast<double>(k)/steps));
      }
    }
    marker_pub_->publish(marker);
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Invalid preview path: %s", error.what());
  }
}

}  // namespace path_planning
