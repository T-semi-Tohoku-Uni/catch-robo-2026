#pragma once

//

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "path_planner/dynamics_model.hpp"
#include "path_planner/math3d.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{

struct TrajectoryFollowerConfig
{
  std::size_t active_axes{4};
  double execution_timeout_sec{30.0};
  double start_state_tolerance{0.02};
  bool enable_simulation_torque{false};
  double lookahead_near_mm{40.0};
  double lookahead_far_mm{150.0};
  double goal_distance_near_mm{100.0};
  double goal_distance_far_mm{500.0};
  double lookahead_smoothing_sec{0.2};

  double goal_position_tolerance_mm{30.0};
  double goal_yaw_tolerance_rad{0.05};
  double goal_joint_tolerance_first_rad{0.05};
  double goal_joint_tolerance_remaining_rad{0.01};

  double rotation_joint_timeout_sec{1.0};
  double wrist_feedback_tolerance_deg{6.0};
  double rotation_speed_rad_sec{1.0};

  double max_tracking_lag_rad{0.25};

  std::array<double, kDof> kp{18.0, 24.0, 24.0, 12.0};
  std::array<double, kDof> kd{4.0, 5.0, 5.0, 2.0};

  void validate() const;
  double wristFeedbackToleranceRad() const;
};

struct RouteWaypoint
{
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct FollowRouteGoal
{
  bool start{true};
  std::string frame_id{"map"};
  std::vector<RouteWaypoint> poses;
  std::uint64_t rotation_group_id{0};
  int wrist_direction{0};
  std::vector<double> wrist_angles;
  std::vector<double> phi_angles;
};

struct FollowerStepOutput
{
  bool finished{false};
  bool success{false};
  bool aborted{false};
  FailureCode failure{FailureCode::kNone};
  std::string message;

  State target_joint_angles;

  State target_joint_velocities;

  State command_torque;

  RouteWaypoint target_pose;
  std::string frame_id{"map"};

  bool use_wrist_control{false};
  std::uint64_t rotation_group_id{0};
  int wrist_direction{0};
  double target_wrist_angle{0.0};

  float route_distance_remaining{0.0f};

  float progress_ratio{0.0f};
  float time_remaining_sec{0.0f};
  float distance_remaining_mm{0.0f};
  State actual_joint_angles;
  State joint_tracking_error;
  double theta1_torque_nm{0.0};
  double position_error_mm{0.0};
  double yaw_error_rad{0.0};
  double max_joint_tracking_error_rad{0.0};
};

namespace wrist_constraints
{
inline constexpr double kFullTurn = 2.0 * kPi;
inline constexpr double kTolerance = 1e-6;

bool isLegalWrist(double angle);
bool isLegalFeedback(double angle, double tolerance_rad);
double clampWrist(double angle);
bool isLegalPhiSegment(
  double x0_mm, double y0_mm, double x1_mm, double y1_mm,
  double phi0_rad, double phi1_rad, int direction = 0);
}  // namespace wrist_constraints

class TrajectoryFollower
{
public:
  explicit TrajectoryFollower(
    TrajectoryFollowerConfig config = TrajectoryFollowerConfig{},
    KinematicChain chain = defaultKinematicChain(),
    std::shared_ptr<const RneaDynamicsModel> dynamics = nullptr);

  const TrajectoryFollowerConfig & config() const {return config_;}
  const KinematicChain & chain() const {return chain_;}

  void updateCurrentJoints(const std::vector<double> & joints, double timestamp_sec);

  void updateLatestRoute(std::string frame_id, std::vector<RouteWaypoint> poses);

  bool hasValidJoints() const;
  bool hasFreshJoints(double now_sec, bool require_wrist_range = false) const;
  State currentJoints() const;
  State currentJointVelocities() const;
  std::string jointFeedbackDiagnostic(double now_sec) const;

  void stepInternalPlant(
    const State & target_positions,
    const State & target_velocities,
    const State & command_torque,
    double dt_sec,
    double now_sec);

  bool validateAndStartRoute(
    const FollowRouteGoal & goal,
    double now_sec,
    std::string * reject_reason = nullptr);

  FollowerStepOutput stepRoute(double now_sec);

  bool startTimedTrajectory(
    const TimedTrajectory & trajectory,
    double now_sec,
    std::string * reject_reason = nullptr);

  FollowerStepOutput stepTimedTrajectory(double now_sec);

  bool isActive() const;
  void cancel();

private:
  enum class ActiveMode
  {
    kIdle,
    kRoute,
    kTimedTrajectory
  };

  TrajectoryFollowerConfig config_;
  KinematicChain chain_;
  std::shared_ptr<const RneaDynamicsModel> dynamics_;

  mutable std::mutex mutex_;

  bool joint_sample_received_{false};
  bool valid_joint_sample_received_{false};
  bool joints_valid_{false};
  std::size_t joint_sample_size_{0};
  double joint_sample_received_at_{0.0};
  double joints_received_at_{0.0};
  std::array<double, kDof> latest_joint_sample_{0.0, 0.0, 0.0, 0.0};
  State current_joint_angles_{0.0, 0.0, 0.0, 0.0};
  State current_joint_velocities_{0.0, 0.0, 0.0, 0.0};

  std::string latest_route_frame_id_{"map"};
  std::vector<RouteWaypoint> latest_route_poses_;

  ActiveMode active_mode_{ActiveMode::kIdle};
  double start_time_sec_{0.0};
  double last_step_time_sec_{0.0};
  bool final_target_sent_{false};
  double final_target_sent_at_{-1.0};
  State final_target_joints_{0.0, 0.0, 0.0, 0.0};
  double max_joint_tracking_error_rad_{0.0};

  FollowRouteGoal active_route_;
  std::size_t current_path_index_{0};
  double commanded_progress_{0.0};
  bool lookahead_initialized_{false};
  double smoothed_lookahead_mm_{40.0};

  TimedTrajectory active_trajectory_;
  double trajectory_ref_time_sec_{0.0};
  double previous_time_scale_{1.0};

  double baseAngleAtRouteProgress(const std::vector<RouteWaypoint> & poses, double progress) const;
  double limitConstrainedRouteProgress(
    const FollowRouteGoal & goal,
    double start_progress,
    double desired_progress,
    double max_step_rad) const;
  bool feedbackFreshLocked(double now_sec) const;
  bool fixedAxesValid(const State &state) const;
  bool checkJointGoalReachedLocked(const State & local_joints) const;
};

}  // namespace path_planner
