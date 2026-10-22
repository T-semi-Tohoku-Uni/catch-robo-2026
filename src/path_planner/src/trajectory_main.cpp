#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "path_planner/collision_checker.hpp"
#include "path_planner/dynamics_model.hpp"
#include "path_planner/math3d.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/trajectory_node.hpp"
#include "path_planner/trajectory_pipeline.hpp"
#include "path_planner/yaml_lite.hpp"

#ifdef PATH_PLANNER_HAS_ROS2
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace
{
path_planner::TrajectoryPipelineConfig makeDemoPipelineConfig()
{
  path_planner::TrajectoryPipelineConfig cfg;
  cfg.hardware_output_enabled = false;
  cfg.candidate_count = 8;
  cfg.base_seed = 42;
  cfg.planning_time_limit = 1.0;
  cfg.distance_weights = {2.0, 1.0, 1.0, 1.0};
  // Demo bounds cover the source pick/shoot samples; these are not hardware limits.
  cfg.joint_bounds = {{-2.8, -1.8, -1.8, -3.2}, {2.8, 1.8, 2.8, 4.2}};
  cfg.shortcut_iterations = 120;
  cfg.max_path_deviation = 0.02;
  cfg.collision_check_resolution = 0.01;
  cfg.collision_margin = 0.005;
  cfg.topp_grid_points = 200;
  cfg.adaptive_grid_max_points = 2000;
  cfg.validation_dt = 0.001;
  cfg.max_velocity = {1.5, 2.0, 2.0, 3.0};
  cfg.max_acceleration = {2.5, 4.0, 4.0, 6.0};
  cfg.max_jerk = {25.0, 40.0, 40.0, 60.0};
  return cfg;
}

path_planner::RobotDynamicsConfig makeDemoDynamicsConfig()
{
  path_planner::RobotDynamicsConfig dyn;
  dyn.source = "simulation_demo";
  dyn.measured_at = "2026-10-20";
  dyn.safety_factor = 0.85;
  dyn.gravity = {0.0, 0.0, -9.80665};

  // Provisional inertias for simulation only.
  const double masses[5] = {0.8, 1.2, 0.9, 0.3, 0.4};
  const double com_z[5] = {0.045, 0.240, 0.240, 0.0, -0.062};
  for (std::size_t i = 0; i < path_planner::kUrdfJointCount; ++i) {
    dyn.links[i].mass = masses[i];
    dyn.links[i].center_of_mass = {0.0, 0.0, com_z[i]};
    dyn.links[i].inertia(0, 0) = 0.02 * masses[i];
    dyn.links[i].inertia(1, 1) = 0.02 * masses[i];
    dyn.links[i].inertia(2, 2) = 0.01 * masses[i];
  }
  dyn.links[0].inertia(2, 2) = 0.18;

  dyn.theta1.continuous_torque = 1.2;
  dyn.theta1.peak_torque = 2.0;
  dyn.theta1.gear_ratio = 10.0;
  dyn.theta1.efficiency = 0.90;
  dyn.theta1.curve_in_motor_side = false;
  dyn.theta1.positive_speed_torque_curve = {{0.0, 2.0}, {1.0, 1.6}, {2.0, 0.8}};
  dyn.theta1.negative_speed_torque_curve = {{0.0, 2.0}, {1.0, 1.6}, {2.0, 0.8}};
  return dyn;
}

void printDemoSummary(
  const path_planner::TrajectoryPipeline & pipeline,
  const path_planner::GenerateTrajectoryResponse & resp,
  const path_planner::DiagnosticArrayMsg & diag)
{
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "=== Trajectory Pipeline Execution Demo ===" << std::endl;
  std::cout << "mode                  : "
            << path_planner::toString(pipeline.operationMode()) << std::endl;
  std::cout << "request_id            : " << resp.request_id << std::endl;
  std::cout << "success               : " << (resp.success ? "true" : "false") << std::endl;
  std::cout << "message               : " << resp.message << std::endl;
  if (diag.key_values.count("candidate_count") && diag.key_values.count("selected_candidate")) {
    std::cout << "candidates_evaluated  : " << diag.key_values.at("candidate_count")
              << " (selected=" << diag.key_values.at("selected_candidate") << ")" << std::endl;
  }
  if (diag.key_values.count("bottleneck")) {
    std::cout << "bottleneck            : " << diag.key_values.at("bottleneck") << std::endl;
  }
  if (diag.key_values.count("duration")) {
    std::cout << "duration [s]          : " << diag.key_values.at("duration") << std::endl;
  }
  const auto dyn = pipeline.dynamicsConfig();
  std::cout << "theta1_peak_torque[Nm]: " << resp.theta1_peak_torque
            << " (allowed_max=" << dyn.theta1.peak_torque * dyn.safety_factor << " Nm)"
            << std::endl;
  std::cout << "theta1_rms_torque [Nm]: " << resp.theta1_rms_torque
            << " (allowed_cont=" << dyn.theta1.continuous_torque * dyn.safety_factor << " Nm)"
            << std::endl;
  std::cout << "min_torque_margin [Nm]: " << resp.minimum_torque_margin << std::endl;
  std::cout << "timed_points_count    : " << resp.trajectory.points.size() << std::endl;

  std::cout << "\n--- Stage Timing [s] ---" << std::endl;
  for (const auto & kv : diag.key_values) {
    if (kv.first.rfind("time_", 0) == 0) {
      std::cout << "  " << std::left << std::setw(28) << kv.first << ": " << kv.second
                << std::endl;
    }
  }

  std::cout << "\n--- Sample Trajectory Points (t, theta1..4, dtheta1, ddtheta1, tau1) ---"
            << std::endl;
  const auto & pts = resp.trajectory.points;
  const std::size_t step = std::max<std::size_t>(1, pts.size() / 8);
  for (std::size_t i = 0; i < pts.size(); i += step) {
    const auto & p = pts[i];
    std::cout << "  t=" << std::setw(6) << p.time_from_start_sec
              << "s | q=[" << std::setw(7) << p.positions[0] << ", "
              << std::setw(7) << p.positions[1] << ", "
              << std::setw(7) << p.positions[2] << ", "
              << std::setw(7) << p.positions[3] << "]"
              << " | dq1=" << std::setw(7) << p.velocities[0]
              << " | ddq1=" << std::setw(7) << p.accelerations[0]
              << " | tau1=" << std::setw(7) << p.effort[0] << " Nm" << std::endl;
  }
  if (!pts.empty()) {
    const auto & p = pts.back();
    std::cout << "  t=" << std::setw(6) << p.time_from_start_sec
              << "s | q=[" << std::setw(7) << p.positions[0] << ", "
              << std::setw(7) << p.positions[1] << ", "
              << std::setw(7) << p.positions[2] << ", "
              << std::setw(7) << p.positions[3] << "]"
              << " | dq1=" << std::setw(7) << p.velocities[0]
              << " | ddq1=" << std::setw(7) << p.accelerations[0]
              << " | tau1=" << std::setw(7) << p.effort[0] << " Nm (END)" << std::endl;
  }
}

#ifdef PATH_PLANNER_HAS_ROS2
std::array<double, 4> matrixToQuaternion(const path_planner::Matrix3 & r)
{
  const double trace = r(0, 0) + r(1, 1) + r(2, 2);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    w = 0.25 / s;
    x = (r(2, 1) - r(1, 2)) * s;
    y = (r(0, 2) - r(2, 0)) * s;
    z = (r(1, 0) - r(0, 1)) * s;
  } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2));
    w = (r(2, 1) - r(1, 2)) / s;
    x = 0.25 * s;
    y = (r(0, 1) + r(1, 0)) / s;
    z = (r(0, 2) + r(2, 0)) / s;
  } else if (r(1, 1) > r(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2));
    w = (r(0, 2) - r(2, 0)) / s;
    x = (r(0, 1) + r(1, 0)) / s;
    y = 0.25 * s;
    z = (r(1, 2) + r(2, 1)) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1));
    w = (r(1, 0) - r(0, 1)) / s;
    x = (r(0, 2) + r(2, 0)) / s;
    y = (r(1, 2) + r(2, 1)) / s;
    z = 0.25 * s;
  }
  return {x, y, z, w};
}

std_msgs::msg::ColorRGBA torqueHeatColor(double ratio, float alpha = 1.0f)
{
  const double c = std::clamp(ratio, 0.0, 1.0);
  std_msgs::msg::ColorRGBA col;
  col.a = alpha;
  if (c < 0.5) {
    const double u = c * 2.0;
    col.r = static_cast<float>(0.15 + 0.85 * u);
    col.g = static_cast<float>(0.90);
    col.b = static_cast<float>(0.25 * (1.0 - u));
  } else {
    const double u = (c - 0.5) * 2.0;
    col.r = 1.0f;
    col.g = static_cast<float>(0.90 * (1.0 - 0.85 * u));
    col.b = static_cast<float>(0.10 * (1.0 - u));
  }
  return col;
}

class Ros2TrajectoryVisualizerNode : public rclcpp::Node
{
public:
  Ros2TrajectoryVisualizerNode()
  : Node("trajectory_pipeline")
  {
    // Default preview: source Pick ID 7 to Pick ID 12.
    const std::vector<double> default_start{-0.7582, 0.7605, 2.1702, -0.8126};
    const std::vector<double> default_goal{0.7582, 0.7605, 2.1702, -2.3290};
    const std::vector<double> default_obs_xyz{0.675, 0.490, 0.145};

    start_angles_ = this->declare_parameter<std::vector<double>>("start_angles", default_start);
    goal_angles_ = this->declare_parameter<std::vector<double>>("goal_angles", default_goal);
    enable_demo_obstacle_ = this->declare_parameter<bool>("enable_demo_obstacle", true);
    obstacle_xyz_ = this->declare_parameter<std::vector<double>>("obstacle_xyz", default_obs_xyz);
    obstacle_radius_ = this->declare_parameter<double>("obstacle_radius", 0.065);
    payload_mass_ = this->declare_parameter<double>("payload_mass", 0.40);
    loop_playback_ = this->declare_parameter<bool>("loop_playback", true);
    playback_speed_ = this->declare_parameter<double>("playback_speed", 1.0);
    pause_at_ends_sec_ = this->declare_parameter<double>("pause_at_ends_sec", 1.0);
    seed_ = static_cast<std::uint32_t>(this->declare_parameter<int>("seed", 42));

    RCLCPP_WARN(get_logger(), "Preview only: using simulation_demo inertias and torque limits; no hardware output");
    auto pipe_cfg = makeDemoPipelineConfig();
    auto dyn_cfg = makeDemoDynamicsConfig();
    pipeline_ = std::make_shared<path_planner::TrajectoryPipeline>(pipe_cfg, dyn_cfg);

    if (payload_mass_ > 0.0) {
      path_planner::Payload payload;
      payload.mass = payload_mass_;
      payload.center_of_mass = {0.0, 0.0, -0.06};
      payload.inertia = {0.002, 0.0, 0.0, 0.002, 0.0, 0.001};
      pipeline_->updatePayload(payload);
    }

    if (enable_demo_obstacle_ && obstacle_xyz_.size() == 3) {
      path_planner::Obstacle obs;
      obs.name = "field_center_obstacle";
      obs.shape.type = path_planner::ShapeType::kSphere;
      obs.shape.radius = obstacle_radius_;
      obs.shape.pose = path_planner::makeTransform(
        {obstacle_xyz_[0], obstacle_xyz_[1], obstacle_xyz_[2]}, {0.0, 0.0, 0.0});
      pipeline_->updateCollisionScene({obs});
    }

    node_adapter_ = std::make_unique<path_planner::TrajectoryNode>(pipeline_);

    const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable();
    geometric_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "geometric_path", transient_qos);
    timed_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "timed_trajectory", transient_qos);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "trajectory_diagnostics", 10);

    urdf_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    abs_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "arm/absolute_joint_states", 10);
    flange_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "trajectory_flange_path", transient_qos);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);

    goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "goal_joint_state", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() != path_planner::kDof) {
          RCLCPP_WARN(
            this->get_logger(),
            "Ignoring /goal_joint_state: expected 4 positions [theta1..theta4]");
          return;
        }
        start_angles_ = current_theta_;
        goal_angles_ = msg->position;
        RCLCPP_INFO(
          this->get_logger(),
          "Received new /goal_joint_state -> planning from [%.3f, %.3f, %.3f, %.3f] to "
          "[%.3f, %.3f, %.3f, %.3f]",
          start_angles_[0], start_angles_[1], start_angles_[2], start_angles_[3],
          goal_angles_[0], goal_angles_[1], goal_angles_[2], goal_angles_[3]);
        planAndPublish();
      });

    current_theta_ = start_angles_;
    planAndPublish();

    last_tick_time_ = this->now();
    anim_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {onAnimationTick();});
  }

private:
  void planAndPublish()
  {
    path_planner::GenerateTrajectoryRequest req;
    req.start_joint_names = {"theta1", "theta2", "theta3", "theta4"};
    req.start_positions = start_angles_;
    req.goal_joint_names = {"theta1", "theta2", "theta3", "theta4"};
    req.goal_positions = goal_angles_;
    req.seed = seed_;

    const auto resp = node_adapter_->handleGenerateTrajectory(req);
    const auto diag = node_adapter_->lastDiagnostics();
    printDemoSummary(*pipeline_, resp, diag);

    playback_time_sec_ = -pause_at_ends_sec_;
    forward_play_ = true;

    publishTrajectoryTopics();
  }

  trajectory_msgs::msg::JointTrajectory toRosJointTrajectory(
    const path_planner::JointTrajectoryMsg & src,
    const rclcpp::Time & stamp) const
  {
    trajectory_msgs::msg::JointTrajectory out;
    out.header.stamp = stamp;
    out.header.frame_id = src.frame_id;
    out.joint_names = src.joint_names;
    out.points.reserve(src.points.size());
    for (const auto & p : src.points) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = p.positions;
      pt.velocities = p.velocities;
      pt.accelerations = p.accelerations;
      pt.effort = p.effort;
      const double sec_floor = std::floor(p.time_from_start_sec);
      pt.time_from_start.sec = static_cast<std::int32_t>(sec_floor);
      pt.time_from_start.nanosec = static_cast<std::uint32_t>(
        std::round((p.time_from_start_sec - sec_floor) * 1e9));
      out.points.push_back(std::move(pt));
    }
    return out;
  }

  void publishTrajectoryTopics()
  {
    const auto now = this->now();
    const auto geom = node_adapter_->lastGeometricPath();
    if (!geom.points.empty()) {
      geometric_pub_->publish(toRosJointTrajectory(geom, now));
    }
    if (node_adapter_->hasPublishedTimedTrajectory()) {
      const auto timed = node_adapter_->lastTimedTrajectory();
      timed_pub_->publish(toRosJointTrajectory(timed, now));
      flange_path_pub_->publish(buildFlangePathMsg(timed, now));
    }
    publishDiagnostics(now);
  }

  void publishDiagnostics(const rclcpp::Time & stamp)
  {
    const auto d = node_adapter_->lastDiagnostics();
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = static_cast<std::uint8_t>(d.level);
    st.name = d.name;
    st.message = d.message;
    st.hardware_id = d.request_id;
    for (const auto & kv : d.key_values) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = kv.first;
      item.value = kv.second;
      st.values.push_back(std::move(item));
    }
    arr.status.push_back(std::move(st));
    diag_pub_->publish(arr);
  }

  nav_msgs::msg::Path buildFlangePathMsg(
    const path_planner::JointTrajectoryMsg & timed,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = "map";
    const auto chain = pipeline_->dynamicsConfig().chain;
    const std::size_t stride = std::max<std::size_t>(1, timed.points.size() / 250);
    for (std::size_t i = 0; i < timed.points.size(); i += stride) {
      const auto & pt = timed.points[i];
      const auto q_u = path_planner::toUrdfJointPositions(pt.positions);
      const auto frames = path_planner::forwardKinematics(chain, q_u);
      const auto & flange = frames[5];
      const auto quat = matrixToQuaternion(flange.rotation);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = flange.translation.x;
      pose.pose.position.y = flange.translation.y;
      pose.pose.position.z = flange.translation.z;
      pose.pose.orientation.x = quat[0];
      pose.pose.orientation.y = quat[1];
      pose.pose.orientation.z = quat[2];
      pose.pose.orientation.w = quat[3];
      path_msg.poses.push_back(std::move(pose));
    }
    if (!timed.points.empty()) {
      const auto & pt = timed.points.back();
      const auto q_u = path_planner::toUrdfJointPositions(pt.positions);
      const auto frames = path_planner::forwardKinematics(chain, q_u);
      const auto & flange = frames[5];
      const auto quat = matrixToQuaternion(flange.rotation);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = flange.translation.x;
      pose.pose.position.y = flange.translation.y;
      pose.pose.position.z = flange.translation.z;
      pose.pose.orientation.x = quat[0];
      pose.pose.orientation.y = quat[1];
      pose.pose.orientation.z = quat[2];
      pose.pose.orientation.w = quat[3];
      path_msg.poses.push_back(std::move(pose));
    }
    return path_msg;
  }

  void onAnimationTick()
  {
    const auto now = this->now();
    const double dt = (now - last_tick_time_).seconds();
    last_tick_time_ = now;

    const auto res = node_adapter_->lastPipelineResult();
    const auto & pts = res.trajectory.points;

    path_planner::DynamicState sample;
    sample.position = start_angles_;
    sample.velocity = {0.0, 0.0, 0.0, 0.0};
    sample.acceleration = {0.0, 0.0, 0.0, 0.0};
    sample.torque = {0.0, 0.0, 0.0, 0.0};
    sample.time = 0.0;

    if (!pts.empty()) {
      const double total_time = pts.back().time;
      if (forward_play_) {
        playback_time_sec_ += dt * std::max(0.1, playback_speed_);
        if (playback_time_sec_ >= total_time + pause_at_ends_sec_) {
          if (loop_playback_) {
            forward_play_ = false;
            playback_time_sec_ = total_time + pause_at_ends_sec_;
          } else {
            playback_time_sec_ = total_time;
          }
        }
      } else {
        playback_time_sec_ -= dt * std::max(0.1, playback_speed_);
        if (playback_time_sec_ <= -pause_at_ends_sec_) {
          forward_play_ = true;
          playback_time_sec_ = -pause_at_ends_sec_;
        }
      }

      const double clamped_t = std::clamp(playback_time_sec_, 0.0, total_time);
      auto it = std::lower_bound(
        pts.begin(), pts.end(), clamped_t,
        [](const path_planner::DynamicState & s, double t) {return s.time < t;});
      if (it == pts.end()) {
        sample = pts.back();
      } else {
        sample = *it;
      }
      current_theta_ = sample.position;
    }

    publishJointStates(sample, now);
    publishMarkers(res, sample, now);

    if (++tick_counter_ % 25 == 0) {
      publishDiagnostics(now);
      if (node_adapter_->hasPublishedTimedTrajectory()) {
        flange_path_pub_->publish(buildFlangePathMsg(node_adapter_->lastTimedTrajectory(), now));
      }
    }
  }

  void publishJointStates(
    const path_planner::DynamicState & sample,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::JointState abs_msg;
    abs_msg.header.stamp = stamp;
    abs_msg.name = {"theta1", "theta2", "theta3", "theta4"};
    abs_msg.position = sample.position;
    abs_msg.velocity = sample.velocity;
    abs_msg.effort = sample.torque;
    abs_joint_pub_->publish(abs_msg);

    const auto q_u = path_planner::toUrdfJointPositions(sample.position);
    const auto dq_u = path_planner::toUrdfJointRates(sample.velocity);
    sensor_msgs::msg::JointState urdf_msg;
    urdf_msg.header.stamp = stamp;
    urdf_msg.name = {
      "base_yaw_joint", "shoulder_joint", "elbow_joint",
      "wrist_level_joint", "flange_yaw_joint"
    };
    urdf_msg.position.assign(q_u.begin(), q_u.end());
    urdf_msg.velocity.assign(dq_u.begin(), dq_u.end());
    urdf_joint_pub_->publish(urdf_msg);
  }

  void publishMarkers(
    const path_planner::PipelineResult & res,
    const path_planner::DynamicState & current_sample,
    const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto dyn_cfg = pipeline_->dynamicsConfig();
    const auto & chain = dyn_cfg.chain;
    const double max_allowed_tau1 =
      std::max(1e-3, dyn_cfg.theta1.peak_torque * dyn_cfg.safety_factor);

    // Color the flange path by simulated first-axis torque.
    if (!res.trajectory.points.empty()) {
      visualization_msgs::msg::Marker flange_line;
      flange_line.header.frame_id = "map";
      flange_line.header.stamp = stamp;
      flange_line.ns = "flange_torque_path";
      flange_line.id = 0;
      flange_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      flange_line.action = visualization_msgs::msg::Marker::ADD;
      flange_line.pose.orientation.w = 1.0;
      flange_line.scale.x = 0.014;

      visualization_msgs::msg::Marker elbow_line = flange_line;
      elbow_line.ns = "elbow_wrist_paths";
      elbow_line.id = 1;
      elbow_line.scale.x = 0.005;
      elbow_line.color.r = 0.4f;
      elbow_line.color.g = 0.8f;
      elbow_line.color.b = 1.0f;
      elbow_line.color.a = 0.55f;

      const auto & pts = res.trajectory.points;
      const std::size_t stride = std::max<std::size_t>(1, pts.size() / 250);
      for (std::size_t i = 0; i < pts.size(); i += stride) {
        const auto & s = pts[i];
        const auto frames = path_planner::forwardKinematics(
          chain, path_planner::toUrdfJointPositions(s.position));
        geometry_msgs::msg::Point pf;
        pf.x = frames[5].translation.x;
        pf.y = frames[5].translation.y;
        pf.z = frames[5].translation.z;
        flange_line.points.push_back(pf);
        const double tau1 = s.torque.empty() ? 0.0 : std::abs(s.torque[0]);
        flange_line.colors.push_back(torqueHeatColor(tau1 / max_allowed_tau1, 0.95f));

        geometry_msgs::msg::Point pe;
        pe.x = frames[3].translation.x;
        pe.y = frames[3].translation.y;
        pe.z = frames[3].translation.z;
        elbow_line.points.push_back(pe);
      }
      const auto & s_last = pts.back();
      const auto frames_last = path_planner::forwardKinematics(
        chain, path_planner::toUrdfJointPositions(s_last.position));
      geometry_msgs::msg::Point pf_last;
      pf_last.x = frames_last[5].translation.x;
      pf_last.y = frames_last[5].translation.y;
      pf_last.z = frames_last[5].translation.z;
      flange_line.points.push_back(pf_last);
      const double tau1_last = s_last.torque.empty() ? 0.0 : std::abs(s_last.torque[0]);
      flange_line.colors.push_back(torqueHeatColor(tau1_last / max_allowed_tau1, 0.95f));

      arr.markers.push_back(std::move(flange_line));
      arr.markers.push_back(std::move(elbow_line));
    }

    // Show all candidate paths.
    for (std::size_t c = 0; c < res.candidate_paths.size(); ++c) {
      const auto & cpath = res.candidate_paths[c];
      if (cpath.size() < 2) {
        continue;
      }
      visualization_msgs::msg::Marker cand_line;
      cand_line.header.frame_id = "map";
      cand_line.header.stamp = stamp;
      cand_line.ns = "candidate_paths";
      cand_line.id = static_cast<int>(10 + c);
      cand_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      cand_line.action = visualization_msgs::msg::Marker::ADD;
      cand_line.pose.orientation.w = 1.0;
      const bool is_sel = (c == res.selected_candidate);
      cand_line.scale.x = is_sel ? 0.006 : 0.003;
      cand_line.color.r = is_sel ? 1.0f : 0.3f;
      cand_line.color.g = is_sel ? 1.0f : 0.8f;
      cand_line.color.b = 1.0f;
      cand_line.color.a = is_sel ? 0.70f : 0.30f;

      for (std::size_t i = 0; i + 1 < cpath.size(); ++i) {
        for (int sub = 0; sub <= 8; ++sub) {
          const double a = static_cast<double>(sub) / 8.0;
          path_planner::State q(path_planner::kDof, 0.0);
          for (std::size_t j = 0; j < path_planner::kDof; ++j) {
            q[j] = (1.0 - a) * cpath[i][j] + a * cpath[i + 1][j];
          }
          const auto frames = path_planner::forwardKinematics(
            chain, path_planner::toUrdfJointPositions(q));
          geometry_msgs::msg::Point p;
          p.x = frames[5].translation.x;
          p.y = frames[5].translation.y;
          p.z = frames[5].translation.z;
          cand_line.points.push_back(p);
        }
      }
      arr.markers.push_back(std::move(cand_line));
    }

    // Show start and goal markers.
    {
      const auto f_start = path_planner::forwardKinematics(
        chain, path_planner::toUrdfJointPositions(start_angles_))[5];
      const auto f_goal = path_planner::forwardKinematics(
        chain, path_planner::toUrdfJointPositions(goal_angles_))[5];

      visualization_msgs::msg::Marker m_start;
      m_start.header.frame_id = "map";
      m_start.header.stamp = stamp;
      m_start.ns = "start_goal";
      m_start.id = 100;
      m_start.type = visualization_msgs::msg::Marker::SPHERE;
      m_start.action = visualization_msgs::msg::Marker::ADD;
      m_start.pose.position.x = f_start.translation.x;
      m_start.pose.position.y = f_start.translation.y;
      m_start.pose.position.z = f_start.translation.z;
      m_start.pose.orientation.w = 1.0;
      m_start.scale.x = 0.045;
      m_start.scale.y = 0.045;
      m_start.scale.z = 0.045;
      m_start.color.r = 0.1f;
      m_start.color.g = 1.0f;
      m_start.color.b = 0.2f;
      m_start.color.a = 0.95f;
      arr.markers.push_back(m_start);

      visualization_msgs::msg::Marker m_goal = m_start;
      m_goal.id = 101;
      m_goal.pose.position.x = f_goal.translation.x;
      m_goal.pose.position.y = f_goal.translation.y;
      m_goal.pose.position.z = f_goal.translation.z;
      m_goal.color.r = 0.2f;
      m_goal.color.g = 0.5f;
      m_goal.color.b = 1.0f;
      arr.markers.push_back(m_goal);
    }

    // Show scene obstacles.
    const auto scene = pipeline_->collisionScene();
    if (scene) {
      for (std::size_t i = 0; i < scene->obstacles.size(); ++i) {
        const auto & obs = scene->obstacles[i];
        visualization_msgs::msg::Marker om;
        om.header.frame_id = "map";
        om.header.stamp = stamp;
        om.ns = "obstacles";
        om.id = static_cast<int>(200 + i);
        om.action = visualization_msgs::msg::Marker::ADD;
        om.pose.position.x = obs.shape.pose.translation.x;
        om.pose.position.y = obs.shape.pose.translation.y;
        om.pose.position.z = obs.shape.pose.translation.z;
        const auto q = matrixToQuaternion(obs.shape.pose.rotation);
        om.pose.orientation.x = q[0];
        om.pose.orientation.y = q[1];
        om.pose.orientation.z = q[2];
        om.pose.orientation.w = q[3];
        if (obs.shape.type == path_planner::ShapeType::kSphere) {
          om.type = visualization_msgs::msg::Marker::SPHERE;
          om.scale.x = 2.0 * obs.shape.radius;
          om.scale.y = 2.0 * obs.shape.radius;
          om.scale.z = 2.0 * obs.shape.radius;
        } else if (obs.shape.type == path_planner::ShapeType::kCapsule) {
          om.type = visualization_msgs::msg::Marker::CYLINDER;
          om.scale.x = 2.0 * obs.shape.radius;
          om.scale.y = 2.0 * obs.shape.radius;
          om.scale.z = obs.shape.length + 2.0 * obs.shape.radius;
        } else {
          om.type = visualization_msgs::msg::Marker::CUBE;
          om.scale.x = 2.0 * obs.shape.half_extents.x;
          om.scale.y = 2.0 * obs.shape.half_extents.y;
          om.scale.z = 2.0 * obs.shape.half_extents.z;
        }
        om.color.r = 1.0f;
        om.color.g = 0.25f;
        om.color.b = 0.15f;
        om.color.a = 0.75f;
        arr.markers.push_back(std::move(om));
      }
    }

    // Show the current flange and simulated torque diagnostics.
    {
      const auto f_cur = path_planner::forwardKinematics(
        chain, path_planner::toUrdfJointPositions(current_sample.position))[5];
      const double cur_tau1 =
        current_sample.torque.empty() ? 0.0 : current_sample.torque[0];

      visualization_msgs::msg::Marker cur_sphere;
      cur_sphere.header.frame_id = "map";
      cur_sphere.header.stamp = stamp;
      cur_sphere.ns = "current_flange";
      cur_sphere.id = 300;
      cur_sphere.type = visualization_msgs::msg::Marker::SPHERE;
      cur_sphere.action = visualization_msgs::msg::Marker::ADD;
      cur_sphere.pose.position.x = f_cur.translation.x;
      cur_sphere.pose.position.y = f_cur.translation.y;
      cur_sphere.pose.position.z = f_cur.translation.z;
      cur_sphere.pose.orientation.w = 1.0;
      cur_sphere.scale.x = 0.036;
      cur_sphere.scale.y = 0.036;
      cur_sphere.scale.z = 0.036;
      cur_sphere.color = torqueHeatColor(std::abs(cur_tau1) / max_allowed_tau1, 1.0f);
      arr.markers.push_back(cur_sphere);

      visualization_msgs::msg::Marker hud;
      hud.header.frame_id = "map";
      hud.header.stamp = stamp;
      hud.ns = "hud_info";
      hud.id = 301;
      hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      hud.action = visualization_msgs::msg::Marker::ADD;
      hud.pose.position.x = 0.675;
      hud.pose.position.y = 0.35;
      hud.pose.position.z = 0.78;
      hud.pose.orientation.w = 1.0;
      hud.scale.z = 0.045;
      hud.color.r = 1.0f;
      hud.color.g = 1.0f;
      hud.color.b = 1.0f;
      hud.color.a = 0.95f;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "t=" << current_sample.time << " / " << res.duration << " s\n"
          << "tau1(t)=" << cur_tau1 << " Nm | peak=" << res.theta1_peak_torque
          << " Nm (max " << max_allowed_tau1 << ")\n"
          << "RMS(tau1)=" << res.theta1_rms_torque << " Nm | cand=" << res.selected_candidate
          << "/" << res.candidate_count << " (" << res.bottleneck << ")";
      hud.text = oss.str();
      arr.markers.push_back(std::move(hud));
    }

    marker_pub_->publish(arr);
  }

  std::vector<double> start_angles_;
  std::vector<double> goal_angles_;
  std::vector<double> current_theta_;
  bool enable_demo_obstacle_{true};
  std::vector<double> obstacle_xyz_;
  double obstacle_radius_{0.065};
  double payload_mass_{0.40};
  bool loop_playback_{true};
  double playback_speed_{1.0};
  double pause_at_ends_sec_{1.0};
  std::uint32_t seed_{42};

  double playback_time_sec_{0.0};
  bool forward_play_{true};
  std::uint64_t tick_counter_{0};
  rclcpp::Time last_tick_time_;

  std::shared_ptr<path_planner::TrajectoryPipeline> pipeline_;
  std::unique_ptr<path_planner::TrajectoryNode> node_adapter_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr geometric_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr timed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr urdf_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr abs_joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr flange_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr anim_timer_;
};
#endif  // PATH_PLANNER_HAS_ROS2

}  // namespace

int main(int argc, char ** argv)
{
  std::string pipeline_yaml = "config/trajectory_pipeline.yaml";
  std::string dynamics_yaml = "config/robot_dynamics.yaml";
  bool run_demo = false;
  bool check_config_only = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--pipeline-config" && i + 1 < argc) {
      pipeline_yaml = argv[++i];
      check_config_only = true;
    } else if (arg == "--dynamics-config" && i + 1 < argc) {
      dynamics_yaml = argv[++i];
      check_config_only = true;
    } else if (arg == "--demo") {
      run_demo = true;
    }
  }

  try {
    if (run_demo) {
      auto pipeline = std::make_shared<path_planner::TrajectoryPipeline>(
        makeDemoPipelineConfig(), makeDemoDynamicsConfig());

      // Demonstrate obstacle avoidance with simulated dynamics.
      path_planner::Obstacle obs;
      obs.name = "field_center_obstacle";
      obs.shape.type = path_planner::ShapeType::kSphere;
      obs.shape.radius = 0.065;
      obs.shape.pose = path_planner::makeTransform({0.675, 0.490, 0.145}, {0.0, 0.0, 0.0});
      pipeline->updateCollisionScene({obs});

      path_planner::TrajectoryNode node(pipeline);
      path_planner::GenerateTrajectoryRequest req;
      req.start_joint_names = {"theta1", "theta2", "theta3", "theta4"};
      req.start_positions = {-0.7582, 0.7605, 2.1702, -0.8126};
      req.goal_joint_names = {"theta1", "theta2", "theta3", "theta4"};
      req.goal_positions = {0.7582, 0.7605, 2.1702, -2.3290};
      req.seed = 42;

      const auto resp = node.handleGenerateTrajectory(req);
      const auto diag = node.lastDiagnostics();
      printDemoSummary(*pipeline, resp, diag);
      return resp.success ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (check_config_only) {
      const path_planner::YamlNode dyn_node = path_planner::parseYamlFile(dynamics_yaml);
      const path_planner::RobotDynamicsConfig dyn_cfg =
        path_planner::loadRobotDynamicsConfig(dyn_node);

      const path_planner::YamlNode pipe_node = path_planner::parseYamlFile(pipeline_yaml);
      const path_planner::TrajectoryPipelineConfig pipe_cfg =
        path_planner::loadTrajectoryPipelineConfig(pipe_node, dyn_cfg.chain);

      auto pipeline = std::make_shared<path_planner::TrajectoryPipeline>(pipe_cfg, dyn_cfg);
      path_planner::TrajectoryNode node(pipeline);
      std::cout << "[trajectory_node] initialized in mode="
                << path_planner::toString(pipeline->operationMode()) << std::endl;
      return EXIT_SUCCESS;
    }

#ifdef PATH_PLANNER_HAS_ROS2
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<Ros2TrajectoryVisualizerNode>();
    rclcpp::spin(ros_node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
#else
    const path_planner::YamlNode dyn_node = path_planner::parseYamlFile(dynamics_yaml);
    const path_planner::RobotDynamicsConfig dyn_cfg =
      path_planner::loadRobotDynamicsConfig(dyn_node);

    const path_planner::YamlNode pipe_node = path_planner::parseYamlFile(pipeline_yaml);
    const path_planner::TrajectoryPipelineConfig pipe_cfg =
      path_planner::loadTrajectoryPipelineConfig(pipe_node, dyn_cfg.chain);

    auto pipeline = std::make_shared<path_planner::TrajectoryPipeline>(pipe_cfg, dyn_cfg);
    path_planner::TrajectoryNode node(pipeline);
    std::cout << "[trajectory_node] initialized in mode="
              << path_planner::toString(pipeline->operationMode()) << std::endl;
    return EXIT_SUCCESS;
#endif
  } catch (const std::exception & ex) {
    std::cerr << "[trajectory_node] startup failed: " << ex.what() << std::endl;
    return EXIT_FAILURE;
  }
}
