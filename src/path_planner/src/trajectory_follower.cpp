#include "path_planner/trajectory_follower.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace path_planner
{
namespace
{
struct Quat4
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

double quatNormSq(const RouteWaypoint & p)
{
  return p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw;
}

Quat4 normalizeQuat(double x, double y, double z, double w)
{
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(n) || n < 1e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {x / n, y / n, z / n, w / n};
}

Matrix3 quatToMatrix(const Quat4 & q)
{
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  Matrix3 m{};
  m(0, 0) = 1.0 - 2.0 * (yy + zz);
  m(0, 1) = 2.0 * (xy - wz);
  m(0, 2) = 2.0 * (xz + wy);
  m(1, 0) = 2.0 * (xy + wz);
  m(1, 1) = 1.0 - 2.0 * (xx + zz);
  m(1, 2) = 2.0 * (yz - wx);
  m(2, 0) = 2.0 * (xz - wy);
  m(2, 1) = 2.0 * (yz + wx);
  m(2, 2) = 1.0 - 2.0 * (xx + yy);
  return m;
}

Quat4 rpyToQuat(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  Quat4 q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return normalizeQuat(q.x, q.y, q.z, q.w);
}

Quat4 slerpQuat(const Quat4 & qa_in, const Quat4 & qb_in, double t)
{
  const Quat4 qa = normalizeQuat(qa_in.x, qa_in.y, qa_in.z, qa_in.w);
  Quat4 qb = normalizeQuat(qb_in.x, qb_in.y, qb_in.z, qb_in.w);
  double dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
  if (dot < 0.0) {
    dot = -dot;
    qb.x = -qb.x;
    qb.y = -qb.y;
    qb.z = -qb.z;
    qb.w = -qb.w;
  }
  if (dot > 0.9995) {
    return normalizeQuat(
      qa.x + t * (qb.x - qa.x),
      qa.y + t * (qb.y - qa.y),
      qa.z + t * (qb.z - qa.z),
      qa.w + t * (qb.w - qa.w));
  }
  const double theta_0 = std::acos(std::clamp(dot, -1.0, 1.0));
  const double theta = theta_0 * t;
  const double sin_theta = std::sin(theta);
  const double sin_theta_0 = std::sin(theta_0);
  const double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  const double s1 = sin_theta / sin_theta_0;
  return normalizeQuat(
    s0 * qa.x + s1 * qb.x,
    s0 * qa.y + s1 * qb.y,
    s0 * qa.z + s1 * qb.z,
    s0 * qa.w + s1 * qb.w);
}

bool extractYawFromWaypoint(const RouteWaypoint & wp, double * out_yaw, double * out_y_hypot = nullptr)
{
  const double n2 = quatNormSq(wp);
  if (!std::isfinite(n2) || n2 < 1e-12) {
    return false;
  }
  const Quat4 q = normalizeQuat(wp.qx, wp.qy, wp.qz, wp.qw);
  const Matrix3 m = quatToMatrix(q);
  const double yx = m(0, 1);
  const double yy = m(1, 1);
  if (out_y_hypot != nullptr) {
    *out_y_hypot = std::hypot(yx, yy);
  }
  *out_yaw = std::atan2(-yx, yy);
  return std::isfinite(*out_yaw);
}

double angleAtProgress(const std::vector<double> & angles, double progress)
{
  if (angles.empty()) {
    return 0.0;
  }
  if (angles.size() == 1) {
    return angles.front();
  }
  const std::size_t idx =
    std::min(static_cast<std::size_t>( std::max(0.0, progress)), angles.size() - 2);
  const double t = progress - static_cast<double>(idx);
  return angles[idx] + (angles[idx + 1] - angles[idx]) * t;
}

double wristAtProgress(const std::vector<double> & angles, double progress)
{
  return wrist_constraints::clampWrist(angleAtProgress(angles, progress));
}

RouteWaypoint poseAtProgress(const std::vector<RouteWaypoint> & poses, double progress)
{
  if (poses.empty()) {
    return RouteWaypoint{};
  }
  if (poses.size() == 1) {
    return poses.front();
  }
  const std::size_t idx =
    std::min(static_cast<std::size_t>(std::max(0.0, progress)), poses.size() - 2);
  const double t = progress - static_cast<double>(idx);
  const RouteWaypoint & a = poses[idx];
  const RouteWaypoint & b = poses[idx + 1];

  RouteWaypoint out;
  out.x_m = a.x_m + t * (b.x_m - a.x_m);
  out.y_m = a.y_m + t * (b.y_m - a.y_m);
  out.z_m = a.z_m + t * (b.z_m - a.z_m);
  const Quat4 q = slerpQuat({a.qx, a.qy, a.qz, a.qw}, {b.qx, b.qy, b.qz, b.qw}, t);
  out.qx = q.x;
  out.qy = q.y;
  out.qz = q.z;
  out.qw = q.w;
  return out;
}

RouteWaypoint buildTargetPoseMsg(double x_mm, double y_mm, double z_mm, double target_yaw_rad)
{
  RouteWaypoint out;
  out.x_m = x_mm / 1000.0;
  out.y_m = y_mm / 1000.0;
  out.z_m = z_mm / 1000.0;
  const Quat4 q = rpyToQuat(0.0, -kPi / 2.0, target_yaw_rad);
  out.qx = q.x;
  out.qy = q.y;
  out.qz = q.z;
  out.qw = q.w;
  return out;
}
}  // namespace

namespace wrist_constraints
{
bool isLegalWrist(double angle)
{
  return std::isfinite(angle) && angle >= -kFullTurn - kTolerance && angle <= kTolerance;
}

bool isLegalFeedback(double angle, double tolerance_rad)
{
  return std::isfinite(angle) && std::isfinite(tolerance_rad) &&
         tolerance_rad >= 0.0 && tolerance_rad < kFullTurn / 2.0 &&
         angle >= -kFullTurn - tolerance_rad - kTolerance &&
         angle <= tolerance_rad + kTolerance;
}

double clampWrist(double angle)
{
  return std::max(-kFullTurn, std::min(0.0, angle));
}

bool isContinuousBaseSegment(double x0_mm, double y0_mm, double x1_mm, double y1_mm)
{
  if (!std::isfinite(x0_mm) || !std::isfinite(y0_mm) ||
    !std::isfinite(x1_mm) || !std::isfinite(y1_mm))
  {
    return false;
  }
  const double dx = x1_mm - x0_mm;
  const double dy = y1_mm - y0_mm;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (x0_mm * dx + y0_mm * dy);
  const double nearest = a > 0.0 ? std::clamp(-b / (2.0 * a), 0.0, 1.0) : 0.0;
  const double nx = x0_mm + nearest * dx;
  const double ny = y0_mm + nearest * dy;
  if (nx * nx + ny * ny < 1e-6 ||
    std::abs(std::atan2(x1_mm, y1_mm) - std::atan2(x0_mm, y0_mm)) > kFullTurn / 2.0)
  {
    return false;
  }
  return true;
}

bool isLegalPhiSegment(
  double x0_mm, double y0_mm, double x1_mm, double y1_mm,
  double phi0_rad, double phi1_rad, int direction)
{
  if (!isContinuousBaseSegment(x0_mm, y0_mm, x1_mm, y1_mm) ||
    !std::isfinite(phi0_rad) || !std::isfinite(phi1_rad) ||
    direction < -1 || direction > 1)
  {
    return false;
  }
  const double dx = x1_mm - x0_mm;
  const double dy = y1_mm - y0_mm;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (x0_mm * dx + y0_mm * dy);
  const double c = x0_mm * x0_mm + y0_mm * y0_mm;
  const double delta = phi1_rad - phi0_rad;
  if (direction != 0) {
    const double nearest = a > 0.0 ? std::clamp(-b / (2.0 * a), 0.0, 1.0) : 0.0;
    for (double t : {0.0, nearest, 1.0}) {
      const double x = x0_mm + t * dx;
      const double y = y0_mm + t * dy;
      const double wrist_deriv = delta - (y0_mm * dx - x0_mm * dy) / (x * x + y * y);
      if (direction * wrist_deriv < -kTolerance) {
        return false;
      }
    }
  }
  const auto check = [&](double t) {
      return isLegalWrist(phi0_rad + t * delta - std::atan2(x0_mm + t * dx, y0_mm + t * dy));
    };
  if (!check(0.0) || !check(1.0)) {
    return false;
  }
  if (a == 0.0 || delta == 0.0) {
    return true;
  }
  const double stationary_c = c - (y0_mm * dx - x0_mm * dy) / delta;
  const double disc = b * b - 4.0 * a * stationary_c;
  if (disc < 0.0) {
    return true;
  }
  const double root = std::sqrt(disc);
  for (double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
    if (t > 0.0 && t < 1.0 && !check(t)) {
      return false;
    }
  }
  return true;
}
}  // namespace wrist_constraints

void TrajectoryFollowerConfig::validate() const
{
  if ((active_axes!=3 && active_axes!=4) || !std::isfinite(execution_timeout_sec) || execution_timeout_sec<=0 ||
      !std::isfinite(start_state_tolerance) || start_state_tolerance<0)
    throw std::invalid_argument("Invalid active axes, execution timeout, or start tolerance");

  if(!std::isfinite(goal_position_tolerance_mm) || goal_position_tolerance_mm<0 ||
     !std::isfinite(goal_yaw_tolerance_rad) || goal_yaw_tolerance_rad<0 ||
     !std::isfinite(max_tracking_lag_rad) || max_tracking_lag_rad<=0)
    throw std::invalid_argument("Invalid Cartesian tolerance or tracking lag");
  for(size_t i=0;i<kDof;++i) if(!std::isfinite(kp[i]) || kp[i]<0 || !std::isfinite(kd[i]) || kd[i]<0)
    throw std::invalid_argument("Simulation gains must be finite and nonnegative");
  if (!std::isfinite(goal_joint_tolerance_first_rad) || goal_joint_tolerance_first_rad < 0.0 ||
    !std::isfinite(goal_joint_tolerance_remaining_rad) || goal_joint_tolerance_remaining_rad < 0.0)
  {
    throw std::invalid_argument("Joint goal tolerances must be finite and nonnegative");
  }
  if (!std::isfinite(wrist_feedback_tolerance_deg) ||
    wrist_feedback_tolerance_deg < 0.0 || wrist_feedback_tolerance_deg >= 180.0)
  {
    throw std::invalid_argument("wrist_feedback_tolerance_deg must be finite in [0, 180)");
  }
  if (!std::isfinite(rotation_joint_timeout_sec) || rotation_joint_timeout_sec <= 0.0 ||
    !std::isfinite(rotation_speed_rad_sec) || rotation_speed_rad_sec <= 0.0)
  {
    throw std::invalid_argument("Invalid rotation group parameters");
  }
  if (!std::isfinite(lookahead_near_mm) || !std::isfinite(lookahead_far_mm) ||
    !std::isfinite(goal_distance_near_mm) || !std::isfinite(goal_distance_far_mm) ||
    !std::isfinite(lookahead_smoothing_sec) || lookahead_near_mm <= 0.0 ||
    lookahead_far_mm < lookahead_near_mm || goal_distance_near_mm < 0.0 ||
    goal_distance_far_mm <= goal_distance_near_mm || lookahead_smoothing_sec <= 0.0)
  {
    throw std::invalid_argument("Invalid lookahead parameters");
  }
}

double TrajectoryFollowerConfig::wristFeedbackToleranceRad() const
{
  return wrist_feedback_tolerance_deg * kPi / 180.0;
}

TrajectoryFollower::TrajectoryFollower(
  TrajectoryFollowerConfig config,
  KinematicChain chain,
  std::shared_ptr<const RneaDynamicsModel> dynamics)
: config_(std::move(config)),
  chain_(std::move(chain)),
  dynamics_(std::move(dynamics))
{
  config_.validate();
  validateKinematicChain(chain_);
}

void TrajectoryFollower::updateCurrentJoints(
  const std::vector<double> & joints,
  double timestamp_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  joint_sample_received_ = true;
  joint_sample_received_at_ = timestamp_sec;
  joint_sample_size_ = joints.size();
  latest_joint_sample_.fill(std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < std::min(joints.size(), kDof); ++i) {
    latest_joint_sample_[i] = joints[i];
  }

  joints_valid_ = joints.size() == kDof && std::isfinite(timestamp_sec) && fixedAxesValid(joints) &&
    std::all_of(
    joints.begin(), joints.begin() + kDof,
    [](double v) {return std::isfinite(v);});

  if (joints_valid_) {
    const double dt = timestamp_sec - joints_received_at_;
    if (valid_joint_sample_received_ && dt > 1e-4 && dt < 0.5) {
      for (std::size_t i = 0; i < kDof; ++i) {
        current_joint_velocities_[i] = (joints[i] - current_joint_angles_[i]) / dt;
      }
    } else {
      current_joint_velocities_.assign(kDof, 0.0);
    }
    for (std::size_t i = 0; i < kDof; ++i) {
      current_joint_angles_[i] = joints[i];
    }
    joints_received_at_ = timestamp_sec;
    valid_joint_sample_received_ = true;
  }
}

void TrajectoryFollower::updateLatestRoute(
  std::string frame_id,
  std::vector<RouteWaypoint> poses)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_route_frame_id_ = frame_id.empty() ? "map" : std::move(frame_id);
  latest_route_poses_ = std::move(poses);
}

bool TrajectoryFollower::hasValidJoints() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return joints_valid_;
}

bool TrajectoryFollower::hasFreshJoints(double now_sec, bool require_wrist_range) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!joints_valid_) {
    return false;
  }
  if (require_wrist_range &&
    !wrist_constraints::isLegalFeedback(
      current_joint_angles_[3], config_.wristFeedbackToleranceRad()))
  {
    return false;
  }
  return feedbackFreshLocked(now_sec);
}

State TrajectoryFollower::currentJoints() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_joint_angles_;
}

State TrajectoryFollower::currentJointVelocities() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_joint_velocities_;
}

std::string TrajectoryFollower::jointFeedbackDiagnostic(double now_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double valid_age = now_sec - joints_received_at_;
  const double tol_rad = config_.wristFeedbackToleranceRad();
  const char * reason =
    !joint_sample_received_ ? "not_received" :
    joint_sample_size_ < kDof ? "too_few_values" :
    !joints_valid_ ? "non_finite" :
    !wrist_constraints::isLegalFeedback(current_joint_angles_[3], tol_rad) ? "wrist_out_of_range" :
    valid_age > config_.rotation_joint_timeout_sec ? "stale" : "ok";

  std::ostringstream detail;
  detail << std::fixed << std::setprecision(6)
         << "reason=" << reason << ", sample_size=" << joint_sample_size_
         << ", last_sample_age_sec=";
  if (joint_sample_received_) {
    detail << (now_sec - joint_sample_received_at_);
  } else {
    detail << "never";
  }
  detail << ", last_valid_age_sec=";
  if (valid_joint_sample_received_) {
    detail << valid_age;
  } else {
    detail << "never";
  }
  detail << ", timeout_sec=" << config_.rotation_joint_timeout_sec << ", q=[";
  for (std::size_t i = 0; i < kDof; ++i) {
    if (i != 0) {
      detail << ',';
    }
    if (joint_sample_received_ && i < joint_sample_size_) {
      detail << std::setprecision(9) << latest_joint_sample_[i];
    } else {
      detail << "missing";
    }
  }
  detail << "], wrist_range=[" << -wrist_constraints::kFullTurn << ",0]"
         << ", wrist_tolerance_rad=" << wrist_constraints::kTolerance
         << ", wrist_feedback_tolerance_deg=" << config_.wrist_feedback_tolerance_deg
         << ", wrist_feedback_range=["
         << -wrist_constraints::kFullTurn - tol_rad << ',' << tol_rad << ']';
  return detail.str();
}

void TrajectoryFollower::stepInternalPlant(
  const State & target_positions,
  const State & target_velocities,
  const State & command_torque,
  double dt_sec,
  double now_sec)
{
  if (!config_.enable_simulation_torque || target_positions.size() != kDof || dt_sec <= 0.0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!joints_valid_) {
    current_joint_angles_ = target_positions;
    current_joint_velocities_.assign(kDof, 0.0);
  } else {

    const double clamped_dt = std::min(dt_sec, 0.05);
    for (std::size_t i = 0; i < kDof; ++i) {
      const double pos_err = target_positions[i] - current_joint_angles_[i];
      const double ref_vel =
        (target_velocities.size() == kDof) ? target_velocities[i] : 0.0;
      const double vel_err = ref_vel - current_joint_velocities_[i];

      double eff_inertia = 0.25;
      if (dynamics_) {
        eff_inertia = std::max(0.05, dynamics_->effectiveInertia(i, current_joint_angles_));
      }
      double acc = (config_.kp[i] * pos_err + config_.kd[i] * vel_err) / eff_inertia;
      if (command_torque.size() == kDof && dynamics_) {
        const State g = dynamics_->gravityTorque(current_joint_angles_);
        acc = (command_torque[i] - g[i]) / eff_inertia;
      }
      current_joint_velocities_[i] += acc * clamped_dt;

      const double alpha = 1.0 - std::exp(-18.0 * clamped_dt);
      current_joint_angles_[i] +=
        (1.0 - alpha) * current_joint_velocities_[i] * clamped_dt + alpha * pos_err;
    }
  }
  if(config_.active_axes==3) {current_joint_angles_[3]=0.0;current_joint_velocities_[3]=0.0;}
  joint_sample_received_ = true;
  valid_joint_sample_received_ = true;
  joints_valid_ = true;
  joint_sample_size_ = kDof;
  joint_sample_received_at_ = now_sec;
  joints_received_at_ = now_sec;
  for (std::size_t i = 0; i < kDof; ++i) {
    latest_joint_sample_[i] = current_joint_angles_[i];
  }
}

double TrajectoryFollower::baseAngleAtRouteProgress(
  const std::vector<RouteWaypoint> & poses,
  double progress) const
{
  const RouteWaypoint wp = poseAtProgress(poses, progress);
  CartesianPose target{wp.x_m * 1000.0, wp.y_m * 1000.0, wp.z_m * 1000.0, 0.0};
  State joints;
  if (!inverseKinematics(chain_, target, joints, true)) {
    throw std::runtime_error("Constrained route target is unreachable");
  }
  return joints[0];
}

double TrajectoryFollower::limitConstrainedRouteProgress(
  const FollowRouteGoal & goal,
  double start_progress,
  double desired_progress,
  double max_step_rad) const
{
  const bool interpolate_phi = !goal.phi_angles.empty();
  double progress = start_progress;
  double budget = max_step_rad;
  while (progress < desired_progress) {
    const double end = std::min(desired_progress, std::floor(progress) + 1.0);
    const auto cost = [&](double target) {
        if (!interpolate_phi) {
          return std::abs(
            wristAtProgress(goal.wrist_angles, target) -
            wristAtProgress(goal.wrist_angles, progress));
        }
        return std::abs(
          angleAtProgress(goal.phi_angles, target) -
          angleAtProgress(goal.phi_angles, progress)) +
               std::abs(
          baseAngleAtRouteProgress(goal.poses, target) -
          baseAngleAtRouteProgress(goal.poses, progress));
      };
    const double segment_cost = cost(end);
    if (segment_cost > budget) {
      double low = progress;
      double high = end;
      for (int iter = 0; iter < 50; ++iter) {
        const double mid = 0.5 * (low + high);
        if (cost(mid) <= budget) {
          low = mid;
        } else {
          high = mid;
        }
      }
      return low;
    }
    budget -= segment_cost;
    progress = end;
  }
  return progress;
}

bool TrajectoryFollower::fixedAxesValid(const State &state) const {
  if(state.size()!=kDof) return false;
  for(size_t i=config_.active_axes;i<kDof;++i) if(state[i]!=0.0) return false;
  return true;
}
bool TrajectoryFollower::feedbackFreshLocked(double now_sec) const {
  const double age=now_sec-joints_received_at_;
  return joints_valid_ && std::isfinite(now_sec) && age>=0.0 && age<=config_.rotation_joint_timeout_sec;
}

bool TrajectoryFollower::checkJointGoalReachedLocked(const State & local_joints) const
{
  if (!joints_valid_ || !final_target_sent_ || joints_received_at_ < final_target_sent_at_) {
    return false;
  }
  for (std::size_t i = 0; i < config_.active_axes; ++i) {
    const double tol = (i == 0) ?
      config_.goal_joint_tolerance_first_rad :
      config_.goal_joint_tolerance_remaining_rad;
    if (std::abs(local_joints[i] - final_target_joints_[i]) > tol) {
      return false;
    }
  }
  return true;
}

bool TrajectoryFollower::validateAndStartRoute(
  const FollowRouteGoal & goal,
  double now_sec,
  std::string * reject_reason)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!goal.start) {
    if (reject_reason) {
      *reject_reason = "goal.start must be true";
    }
    return false;
  }
  if (active_mode_ != ActiveMode::kIdle) {
    if (reject_reason) {
      *reject_reason = "follower is already active";
    }
    return false;
  }

  if(!feedbackFreshLocked(now_sec)) {if(reject_reason) *reject_reason="Fresh current joints required";return false;}
  if(config_.active_axes==3 && goal.rotation_group_id!=0) {if(reject_reason) *reject_reason="Wrist control unavailable with three physical axes";return false;}
  const bool constrained = (goal.rotation_group_id != 0);
  if (constrained) {
    if (goal.poses.empty() || goal.wrist_angles.size() != goal.poses.size() ||
      goal.wrist_direction < -1 || goal.wrist_direction > 1 ||
      (!goal.phi_angles.empty() && goal.phi_angles.size() != goal.poses.size()))
    {
      if (reject_reason) {
        *reject_reason = "invalid constrained route array sizes or wrist_direction";
      }
      return false;
    }
    if (!std::all_of(
        goal.phi_angles.begin(), goal.phi_angles.end(),
        [](double v) {return std::isfinite(v);}))
    {
      if (reject_reason) {
        *reject_reason = "phi_angles contains non-finite value";
      }
      return false;
    }
    for (std::size_t i = 0; i < goal.wrist_angles.size(); ++i) {
      if (!wrist_constraints::isLegalWrist(goal.wrist_angles[i]) ||
        (i > 0 && goal.wrist_direction * (goal.wrist_angles[i] - goal.wrist_angles[i - 1]) <
        -wrist_constraints::kTolerance))
      {
        if (reject_reason) {
          *reject_reason = "wrist_angles violates wrist limits or monotonicity";
        }
        return false;
      }
    }
    const double tol_rad = config_.wristFeedbackToleranceRad();
    const bool fresh = joints_valid_ &&
      wrist_constraints::isLegalFeedback(current_joint_angles_[3], tol_rad) &&
      (now_sec - joints_received_at_) <= config_.rotation_joint_timeout_sec;
    if (!fresh) {
      if (reject_reason) {
        *reject_reason = "fresh current_joints required at sequence group start";
      }
      return false;
    }
    if (std::abs(
        wrist_constraints::clampWrist(current_joint_angles_[3]) - goal.wrist_angles.front()) > 0.05)
    {
      if (reject_reason) {
        *reject_reason = "initial wrist angle does not match wrist_angles.front()";
      }
      return false;
    }
  } else if (!goal.wrist_angles.empty() || !goal.phi_angles.empty() || goal.wrist_direction != 0) {
    if (reject_reason) {
      *reject_reason = "wrist_angles/phi_angles/wrist_direction require non-zero rotation_group_id";
    }
    return false;
  }

  FollowRouteGoal resolved = goal;
  if (resolved.poses.empty()) {
    resolved.frame_id = latest_route_frame_id_;
    resolved.poses = latest_route_poses_;
  }
  if (resolved.poses.empty()) {
    if (reject_reason) {
      *reject_reason = "route path is empty";
    }
    return false;
  }

  for (const auto & pose : resolved.poses) {
    const double n2 = quatNormSq(pose);
    if (!std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) || !std::isfinite(pose.z_m) ||
      !std::isfinite(n2) || n2 < 1e-12)
    {
      if (reject_reason) {
        *reject_reason = "route contains non-finite pose or zero quaternion";
      }
      return false;
    }
  }

  if (!resolved.phi_angles.empty()) {
    const double base_x_mm = chain_.world_to_base.translation.x * 1000.0;
    const double base_y_mm = chain_.world_to_base.translation.y * 1000.0;
    try {
      for (std::size_t i = 0; i < resolved.poses.size(); ++i) {
        const double base = baseAngleAtRouteProgress(resolved.poses, static_cast<double>(i));
        double pose_phi = 0.0;
        double y_hypot = 0.0;
        if (!extractYawFromWaypoint(resolved.poses[i], &pose_phi, &y_hypot) || y_hypot < 1e-6 ||
          std::abs(std::remainder(resolved.phi_angles[i] - pose_phi, 2.0 * kPi)) > 1e-4 ||
          std::abs(resolved.phi_angles[i] - base - resolved.wrist_angles[i]) > 1e-4)
        {
          if (reject_reason) {
            *reject_reason = "phi_angles inconsistent with pose orientation or wrist_angles";
          }
          return false;
        }
        const auto & a = resolved.poses[i == 0 ? i : i - 1];
        const auto & b = resolved.poses[i];
        if (!wrist_constraints::isLegalPhiSegment(
            a.x_m * 1000.0 - base_x_mm, a.y_m * 1000.0 - base_y_mm,
            b.x_m * 1000.0 - base_x_mm, b.y_m * 1000.0 - base_y_mm,
            resolved.phi_angles[i == 0 ? i : i - 1], resolved.phi_angles[i],
            resolved.wrist_direction))
        {
          if (reject_reason) {
            *reject_reason = "phi segment violates wrist or base branch constraints";
          }
          return false;
        }
      }
    } catch (const std::exception & ex) {
      if (reject_reason) {
        *reject_reason = ex.what();
      }
      return false;
    }
  }

  active_route_ = std::move(resolved);
  active_mode_ = ActiveMode::kRoute;
  start_time_sec_ = now_sec;
  last_step_time_sec_ = now_sec;
  current_path_index_ = 0;
  commanded_progress_ = 0.0;
  lookahead_initialized_ = false;
  smoothed_lookahead_mm_ = config_.lookahead_near_mm;
  final_target_sent_ = false;
  final_target_sent_at_ = -1.0;
  max_joint_tracking_error_rad_ = 0.0;
  return true;
}

FollowerStepOutput TrajectoryFollower::stepRoute(double now_sec)
{
  std::unique_lock<std::mutex> lock(mutex_);
  FollowerStepOutput out;
  if (active_mode_ != ActiveMode::kRoute || active_route_.poses.empty()) {
    out.finished = true;
    out.aborted = true;
    out.failure = FailureCode::kInvalidInput;
    out.message = "route follower is not active";
    return out;
  }

  if(!feedbackFreshLocked(now_sec) || now_sec-start_time_sec_>config_.execution_timeout_sec) {
    active_mode_=ActiveMode::kIdle;out.finished=true;out.aborted=true;out.failure=FailureCode::kInvalidInput;
    out.message="Joint feedback stale/invalid or execution deadline exceeded";return out;
  }
  const bool constrained = (active_route_.rotation_group_id != 0);
  if (constrained) {
    const double tol_rad = config_.wristFeedbackToleranceRad();
    const bool fresh = joints_valid_ &&
      wrist_constraints::isLegalFeedback(current_joint_angles_[3], tol_rad) &&
      (now_sec - joints_received_at_) <= config_.rotation_joint_timeout_sec;
    if (!fresh) {
      active_mode_ = ActiveMode::kIdle;
      out.finished = true;
      out.aborted = true;
      out.failure = FailureCode::kJointLimitViolation;
      out.message = "current_joints rejected during sequence group";
      return out;
    }
  }

  const State local_joints = current_joint_angles_;
  const bool joint_goal_reached = checkJointGoalReachedLocked(local_joints);

  const CartesianPose current_pose = forwardKinematicsPose(chain_, local_joints);
  const double cx_mm = current_pose.x_mm;
  const double cy_mm = current_pose.y_mm;
  const double cz_mm = current_pose.z_mm;

  const auto & poses = active_route_.poses;
  const RouteWaypoint & final_wp = poses.back();
  const double goal_distance_mm = std::hypot(
    std::hypot(final_wp.x_m * 1000.0 - cx_mm, final_wp.y_m * 1000.0 - cy_mm),
    final_wp.z_m * 1000.0 - cz_mm);

  const double ratio = std::clamp(
    (goal_distance_mm - config_.goal_distance_near_mm) /
    (config_.goal_distance_far_mm - config_.goal_distance_near_mm),
    0.0, 1.0);
  const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
  const double desired_lookahead_mm =
    config_.lookahead_near_mm + blend * (config_.lookahead_far_mm - config_.lookahead_near_mm);

  const double dt_sec = std::max(0.0, now_sec - last_step_time_sec_);
  const double rotation_dt = std::clamp(dt_sec, 0.0, 0.05);
  if (!lookahead_initialized_) {
    lookahead_initialized_ = true;
    smoothed_lookahead_mm_ = desired_lookahead_mm;
  } else {
    smoothed_lookahead_mm_ +=
      (1.0 - std::exp(-dt_sec / config_.lookahead_smoothing_sec)) *
      (desired_lookahead_mm - smoothed_lookahead_mm_);
  }
  last_step_time_sec_ = now_sec;

  double nearest_dist_sq = std::numeric_limits<double>::max();
  double segment_fraction = 0.0;
  if (poses.size() == 1) {
    current_path_index_ = 0;
  } else {
    for (std::size_t i = current_path_index_; i + 1 < poses.size(); ++i) {
      const auto & a = poses[i];
      const auto & b = poses[i + 1];
      const double vx = (b.x_m - a.x_m) * 1000.0;
      const double vy = (b.y_m - a.y_m) * 1000.0;
      const double vz = (b.z_m - a.z_m) * 1000.0;
      const double len_sq = vx * vx + vy * vy + vz * vz;
      const double t = (len_sq > 0.0) ? std::clamp(
        ((cx_mm - a.x_m * 1000.0) * vx +
        (cy_mm - a.y_m * 1000.0) * vy +
        (cz_mm - a.z_m * 1000.0) * vz) / len_sq,
        0.0, 1.0) : 0.0;
      const double dx = a.x_m * 1000.0 + t * vx - cx_mm;
      const double dy = a.y_m * 1000.0 + t * vy - cy_mm;
      const double dz = a.z_m * 1000.0 + t * vz - cz_mm;
      const double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < nearest_dist_sq) {
        nearest_dist_sq = d2;
        current_path_index_ = i;
        segment_fraction = t;
      }
    }
  }

  RouteWaypoint target_wp = final_wp;
  bool targeting_final = true;
  double target_progress = static_cast<double>(poses.size() - 1);
  double remaining_mm = smoothed_lookahead_mm_;
  for (std::size_t i = current_path_index_; i + 1 < poses.size(); ++i) {
    const auto & a = poses[i];
    const auto & b = poses[i + 1];
    const double length_mm = 1000.0 * std::hypot(
      std::hypot(b.x_m - a.x_m, b.y_m - a.y_m), b.z_m - a.z_m);
    const double start_frac = (i == current_path_index_) ? segment_fraction : 0.0;
    const double available_mm = length_mm * (1.0 - start_frac);
    if (length_mm > 0.0 && remaining_mm < available_mm) {
      const double t = start_frac + remaining_mm / length_mm;
      target_wp.x_m = a.x_m + t * (b.x_m - a.x_m);
      target_wp.y_m = a.y_m + t * (b.y_m - a.y_m);
      target_wp.z_m = a.z_m + t * (b.z_m - a.z_m);
      const Quat4 q = slerpQuat({a.qx, a.qy, a.qz, a.qw}, {b.qx, b.qy, b.qz, b.qw}, t);
      target_wp.qx = q.x;
      target_wp.qy = q.y;
      target_wp.qz = q.z;
      target_wp.qw = q.w;
      target_progress = static_cast<double>(i) + t;
      targeting_final = false;
      break;
    }
    remaining_mm -= available_mm;
  }

  double target_wrist = 0.0;
  try {
    if (constrained) {
      target_progress = std::max(commanded_progress_, target_progress);
      const double max_step = config_.rotation_speed_rad_sec * std::max(rotation_dt, 1e-3);
      target_progress = limitConstrainedRouteProgress(
        active_route_, commanded_progress_, target_progress, max_step);
      target_wrist = active_route_.phi_angles.empty() ?
        wristAtProgress(active_route_.wrist_angles, target_progress) :
        angleAtProgress(active_route_.phi_angles, target_progress) -
        baseAngleAtRouteProgress(poses, target_progress);
      target_wp = poseAtProgress(poses, target_progress);
      targeting_final =
        target_progress >= static_cast<double>(poses.size() - 1) - 1e-9;
    }
  } catch (const std::exception & ex) {
    active_mode_ = ActiveMode::kIdle;
    out.finished = true;
    out.aborted = true;
    out.failure = FailureCode::kPlanningFailed;
    out.message = ex.what();
    return out;
  }

  double target_yaw = 0.0;
  extractYawFromWaypoint(target_wp, &target_yaw);
  CartesianPose target_cart{
    target_wp.x_m * 1000.0,
    target_wp.y_m * 1000.0,
    target_wp.z_m * 1000.0,
    target_yaw};
  State target_joints;
  if (!inverseKinematics(chain_, target_cart, target_joints, !constrained)) {
    active_mode_ = ActiveMode::kIdle;
    out.finished = true;
    out.aborted = true;
    out.failure = FailureCode::kJointLimitViolation;
    out.message = "Route target is unreachable by inverse kinematics";
    return out;
  }
  if (constrained) {
    target_joints[3] = wrist_constraints::clampWrist(target_wrist);
    target_yaw = target_joints[0] + target_joints[3];
  }

  if(config_.active_axes==3) target_joints[3]=0.0;
  const double err_yaw = std::atan2(
    std::sin(target_yaw - current_pose.phi_rad),
    std::cos(target_yaw - current_pose.phi_rad));

  if (targeting_final && joint_goal_reached &&
    goal_distance_mm <= config_.goal_position_tolerance_mm &&
    (config_.active_axes==3 || (config_.active_axes==3 || std::abs(err_yaw) <= config_.goal_yaw_tolerance_rad)) &&
    (!constrained ||
    (commanded_progress_ >= static_cast<double>(poses.size() - 1) - 1e-9 &&
    std::abs(local_joints[3] - active_route_.wrist_angles.back()) <= 0.05)))
  {
    active_mode_ = ActiveMode::kIdle;
    out.finished = true;
    out.success = true;
    out.message = "Reached the end of the route within position, yaw, and joint tolerances";
    out.target_joint_angles = final_target_joints_;
    out.target_joint_velocities = {0.0, 0.0, 0.0, 0.0};
    out.command_torque = {0.0, 0.0, 0.0, 0.0};
    out.target_pose = buildTargetPoseMsg(
      target_cart.x_mm, target_cart.y_mm, target_cart.z_mm, target_yaw);
    out.frame_id = active_route_.frame_id;
    out.route_distance_remaining = 0.0f;
    out.progress_ratio = 1.0f;
    out.distance_remaining_mm = static_cast<float>(goal_distance_mm);
    out.position_error_mm = goal_distance_mm;
    out.yaw_error_rad = std::abs(err_yaw);
    out.max_joint_tracking_error_rad = max_joint_tracking_error_rad_;
    return out;
  }

  if (!targeting_final) {
    final_target_sent_ = false;
  } else if (!final_target_sent_) {
    double final_yaw = 0.0;
    extractYawFromWaypoint(final_wp, &final_yaw);
    CartesianPose final_cart{
      final_wp.x_m * 1000.0, final_wp.y_m * 1000.0, final_wp.z_m * 1000.0, final_yaw};
    if (!inverseKinematics(chain_, final_cart, final_target_joints_, !constrained)) {
      active_mode_ = ActiveMode::kIdle;
      out.finished = true;
      out.aborted = true;
      out.failure = FailureCode::kJointLimitViolation;
      out.message = "Final route target is unreachable";
      return out;
    }
    if (constrained) {
      final_target_joints_[3] = wrist_constraints::clampWrist(active_route_.wrist_angles.back());
    }
    if(config_.active_axes==3) final_target_joints_[3]=0.0;
    final_target_sent_ = true;
    final_target_sent_at_ = now_sec;
  }

  if (constrained) {
    commanded_progress_ = target_progress;
    out.use_wrist_control = true;
    out.rotation_group_id = active_route_.rotation_group_id;
    out.wrist_direction = active_route_.wrist_direction;
    out.target_wrist_angle = target_wrist;
  }

  out.target_joint_angles = target_joints;
  out.target_joint_velocities = {0.0, 0.0, 0.0, 0.0};
  out.command_torque.assign(kDof, 0.0);
  out.actual_joint_angles = local_joints;
  out.joint_tracking_error.assign(kDof, 0.0);
  for (std::size_t i = 0; i < kDof; ++i) {
    out.joint_tracking_error[i] = target_joints[i] - local_joints[i];
    max_joint_tracking_error_rad_ = std::max(
      max_joint_tracking_error_rad_, std::abs(out.joint_tracking_error[i]));
    out.command_torque[i] =
      config_.enable_simulation_torque ? config_.kp[i] * out.joint_tracking_error[i] - config_.kd[i] * current_joint_velocities_[i] : 0.0;
  }
  out.target_pose = buildTargetPoseMsg(
    target_cart.x_mm, target_cart.y_mm, target_cart.z_mm, target_yaw);
  if(!config_.enable_simulation_torque) out.command_torque.clear();
  out.frame_id = active_route_.frame_id;
  out.route_distance_remaining =
    static_cast<float>(poses.size() - current_path_index_);
  out.progress_ratio = poses.size() > 1 ?
    static_cast<float>(target_progress / static_cast<double>(poses.size() - 1)) : 1.0f;
  out.distance_remaining_mm = static_cast<float>(goal_distance_mm);
  out.position_error_mm = goal_distance_mm;
  out.yaw_error_rad = std::abs(err_yaw);
  out.max_joint_tracking_error_rad = max_joint_tracking_error_rad_;
  return out;
}

bool TrajectoryFollower::startTimedTrajectory(
  const TimedTrajectory & trajectory,
  double now_sec,
  std::string * reject_reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_mode_ != ActiveMode::kIdle) {
    if (reject_reason) {
      *reject_reason = "follower is already active";
    }
    return false;
  }
  if (trajectory.points.size() < 2) {
    if (reject_reason) {
      *reject_reason = "timed trajectory must contain at least 2 points";
    }
    return false;
  }
  if(!feedbackFreshLocked(now_sec)) {if(reject_reason) *reject_reason="Fresh current joints required";return false;}
  double prev_t = -1.0;
  for (const auto & pt : trajectory.points) {
    if (!isFiniteState(pt.position) || !fixedAxesValid(pt.position) || !std::isfinite(pt.time) || pt.time<0 || pt.time <= prev_t) {
      if (reject_reason) {
        *reject_reason = "timed trajectory contains invalid state or non-monotonic time";
      }
      return false;
    }
    for(const auto *values:{&pt.velocity,&pt.acceleration,&pt.jerk,&pt.torque}) {
      if(!values->empty() && (!isFiniteState(*values) || (values!=&pt.torque && !fixedAxesValid(*values)))) {
        if(reject_reason) *reject_reason="Invalid trajectory derivative/effort or nonzero fixed axis";return false;
      }
    }
    prev_t = pt.time;
  }
  if(trajectory.points.front().time!=0.0) {if(reject_reason) *reject_reason="Trajectory must start at zero time";return false;}
  for(size_t i=0;i<config_.active_axes;++i) if(std::abs(trajectory.points.front().position[i]-current_joint_angles_[i])>config_.start_state_tolerance) {
    if(reject_reason) *reject_reason="Current position differs from trajectory start";return false;
  }

  active_trajectory_ = trajectory;
  active_mode_ = ActiveMode::kTimedTrajectory;
  start_time_sec_ = now_sec;
  last_step_time_sec_ = now_sec;
  trajectory_ref_time_sec_ = 0.0;
  previous_time_scale_=1.0;
  final_target_joints_ = trajectory.points.back().position;
  final_target_sent_ = false;
  final_target_sent_at_ = -1.0;
  max_joint_tracking_error_rad_ = 0.0;
  return true;
}

FollowerStepOutput TrajectoryFollower::stepTimedTrajectory(double now_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  FollowerStepOutput out;
  if (active_mode_ != ActiveMode::kTimedTrajectory || active_trajectory_.points.empty()) {
    out.finished = true;
    out.aborted = true;
    out.failure = FailureCode::kInvalidInput;
    out.message = "timed trajectory follower is not active";
    return out;
  }

  if(!feedbackFreshLocked(now_sec) || now_sec-start_time_sec_>config_.execution_timeout_sec) {
    active_mode_=ActiveMode::kIdle;out.finished=true;out.aborted=true;out.failure=FailureCode::kInvalidInput;
    out.message="Joint feedback stale/invalid or execution deadline exceeded";return out;
  }
  const auto & pts = active_trajectory_.points;
  const double total_duration = pts.back().time;
  const double dt_sec = std::clamp(now_sec - last_step_time_sec_, 0.0, 0.1);
  last_step_time_sec_ = now_sec;

  const State local_joints = joints_valid_ ? current_joint_angles_ : pts.front().position;
  const State local_vels = joints_valid_ ? current_joint_velocities_ : State{0.0, 0.0, 0.0, 0.0};

  double time_scale = 1.0;
  if (joints_valid_ && config_.max_tracking_lag_rad > 1e-6) {
    auto it_cur = std::lower_bound(
      pts.begin(), pts.end(), trajectory_ref_time_sec_,
      [](const DynamicState & s, double t) {return s.time < t;});
    const State & ref_q = (it_cur == pts.end()) ? pts.back().position : it_cur->position;
    double max_err = 0.0;
    for (std::size_t i = 0; i < kDof; ++i) {
      max_err = std::max(max_err, std::abs(ref_q[i] - local_joints[i]));
    }
    if (max_err > config_.max_tracking_lag_rad) {
      time_scale = std::clamp(
        1.0 - (max_err - config_.max_tracking_lag_rad) / config_.max_tracking_lag_rad,
        0.0, 1.0);
    }
  }

  trajectory_ref_time_sec_ = std::min(
    total_duration, trajectory_ref_time_sec_ + dt_sec * time_scale);
  const bool targeting_final = (trajectory_ref_time_sec_ >= total_duration - 1e-9);

  DynamicState ref_state = pts.back();
  if (trajectory_ref_time_sec_ <= pts.front().time) {
    ref_state = pts.front();
  } else if (!targeting_final) {
    auto it = std::lower_bound(
      pts.begin(), pts.end(), trajectory_ref_time_sec_,
      [](const DynamicState & s, double t) {return s.time < t;});
    if (it != pts.end() && it != pts.begin()) {
      const DynamicState & s1 = *it;
      const DynamicState & s0 = *(it - 1);
      const double seg_dt = std::max(1e-9, s1.time - s0.time);
      const double alpha = std::clamp((trajectory_ref_time_sec_ - s0.time) / seg_dt, 0.0, 1.0);
      ref_state.time = trajectory_ref_time_sec_;
      ref_state.position.assign(kDof, 0.0);
      ref_state.velocity.assign(kDof, 0.0);
      ref_state.acceleration.assign(kDof, 0.0);
      ref_state.torque.assign(kDof, 0.0);
      for (std::size_t i = 0; i < kDof; ++i) {
        ref_state.position[i] = (1.0 - alpha) * s0.position[i] + alpha * s1.position[i];
        if (s0.velocity.size() == kDof && s1.velocity.size() == kDof) {
          ref_state.velocity[i] = (1.0 - alpha) * s0.velocity[i] + alpha * s1.velocity[i];
        }
        if (s0.acceleration.size() == kDof && s1.acceleration.size() == kDof) {
          ref_state.acceleration[i] =
            (1.0 - alpha) * s0.acceleration[i] + alpha * s1.acceleration[i];
        }
        if (s0.torque.size() == kDof && s1.torque.size() == kDof) {
          ref_state.torque[i] = (1.0 - alpha) * s0.torque[i] + alpha * s1.torque[i];
        }
      }
    }
  }

  const double scale_rate=dt_sec>1e-9?(time_scale-previous_time_scale_)/dt_sec:0.0;
  previous_time_scale_=time_scale;
  for(size_t i=0;i<kDof;++i) {
    const double velocity=ref_state.velocity.size()==kDof?ref_state.velocity[i]:0.0;
    if(ref_state.acceleration.size()==kDof)
      ref_state.acceleration[i]=ref_state.acceleration[i]*time_scale*time_scale+velocity*scale_rate;
    if(ref_state.velocity.size()==kDof) ref_state.velocity[i]=velocity*time_scale;
  }
  State tau_ff = ref_state.torque;
  if(config_.enable_simulation_torque && dynamics_) tau_ff.clear();
  if(!config_.enable_simulation_torque) tau_ff.assign(kDof,0.0);
  if (tau_ff.size() != kDof) {
    if (dynamics_) {
      const State zero4{0.0, 0.0, 0.0, 0.0};
      tau_ff = dynamics_->inverseDynamics(
        ref_state.position,
        ref_state.velocity.size() == kDof ? ref_state.velocity : zero4,
        ref_state.acceleration.size() == kDof ? ref_state.acceleration : zero4);
    } else {
      tau_ff.assign(kDof, 0.0);
    }
  }

  out.target_joint_angles = ref_state.position;
  out.target_joint_velocities =
    ref_state.velocity.size() == kDof ? ref_state.velocity : State{0.0, 0.0, 0.0, 0.0};
  out.actual_joint_angles = local_joints;
  out.joint_tracking_error.assign(kDof, 0.0);
  out.command_torque.assign(kDof, 0.0);

  for (std::size_t i = 0; i < kDof; ++i) {
    const double e_q = out.target_joint_angles[i] - local_joints[i];
    const double e_dq = out.target_joint_velocities[i] - local_vels[i];
    out.joint_tracking_error[i] = e_q;
    max_joint_tracking_error_rad_ = std::max(max_joint_tracking_error_rad_, std::abs(e_q));
    out.command_torque[i] = config_.enable_simulation_torque ? tau_ff[i] + config_.kp[i] * e_q + config_.kd[i] * e_dq : 0.0;
  }
  out.theta1_torque_nm = config_.enable_simulation_torque?out.command_torque[0]:0.0;
  if(!config_.enable_simulation_torque) out.command_torque.clear();

  const CartesianPose current_cart = forwardKinematicsPose(chain_, local_joints);
  const CartesianPose ref_cart = forwardKinematicsPose(chain_, out.target_joint_angles);
  const CartesianPose goal_cart = forwardKinematicsPose(chain_, final_target_joints_);

  const double goal_distance_mm = std::hypot(
    std::hypot(goal_cart.x_mm - current_cart.x_mm, goal_cart.y_mm - current_cart.y_mm),
    goal_cart.z_mm - current_cart.z_mm);
  const double err_yaw = std::atan2(
    std::sin(goal_cart.phi_rad - current_cart.phi_rad),
    std::cos(goal_cart.phi_rad - current_cart.phi_rad));

  out.target_pose = buildTargetPoseMsg(
    ref_cart.x_mm, ref_cart.y_mm, ref_cart.z_mm, ref_cart.phi_rad);
  out.frame_id = chain_.world_frame;
  out.progress_ratio = total_duration > 1e-9 ?
    static_cast<float>(std::clamp(trajectory_ref_time_sec_ / total_duration, 0.0, 1.0)) : 1.0f;
  out.time_remaining_sec =
    static_cast<float>(std::max(0.0, total_duration - trajectory_ref_time_sec_));
  out.distance_remaining_mm = static_cast<float>(goal_distance_mm);
  out.route_distance_remaining = out.distance_remaining_mm;
  out.position_error_mm = goal_distance_mm;
  out.yaw_error_rad = std::abs(err_yaw);
  out.max_joint_tracking_error_rad = max_joint_tracking_error_rad_;

  const bool joint_goal_reached = checkJointGoalReachedLocked(local_joints);
  if (targeting_final && joint_goal_reached &&
    goal_distance_mm <= config_.goal_position_tolerance_mm &&
    (config_.active_axes==3 || std::abs(err_yaw) <= config_.goal_yaw_tolerance_rad))
  {
    active_mode_ = ActiveMode::kIdle;
    out.finished = true;
    out.success = true;
    out.message = "Timed trajectory tracking completed within position, yaw, and joint tolerances";
    return out;
  }

  if (targeting_final && !final_target_sent_) {
    final_target_sent_ = true;
    final_target_sent_at_ = now_sec;
  }

  return out;
}

bool TrajectoryFollower::isActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_mode_ != ActiveMode::kIdle;
}

void TrajectoryFollower::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_mode_ = ActiveMode::kIdle;
  final_target_sent_ = false;
}

}  // namespace path_planner
