#include "path_planner/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace path_planner
{
namespace
{
constexpr double kTol = 1e-6;

bool isFiniteVec(const State & v)
{
  if (v.size() != kDof) {
    return false;
  }
  for (double x : v) {
    if (!std::isfinite(x)) {
      return false;
    }
  }
  return true;
}

double maxAbsDiff(const State & a, const State & b)
{
  double m = 0.0;
  for (std::size_t i = 0; i < kDof; ++i) {
    m = std::max(m, std::abs(a[i] - b[i]));
  }
  return m;
}
}  // namespace

double computeTheta1RmsTorque(
  const TimedTrajectory & trajectory,
  double cycle_period,
  double rest_torque)
{
  const auto & pts = trajectory.points;
  if (pts.size() < 2) {
    return 0.0;
  }
  const double duration = pts.back().time - pts.front().time;
  if (!std::isfinite(duration) || duration <= 0.0) {
    return 0.0;
  }
  double sq_integral = 0.0;
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    const double dt = pts[i + 1].time - pts[i].time;
    const double t0 = pts[i].torque.empty() ? 0.0 : pts[i].torque[0];
    const double t1 = pts[i + 1].torque.empty() ? 0.0 : pts[i + 1].torque[0];
    sq_integral += 0.5 * (t0 * t0 + t1 * t1) * dt;
  }
  double period = duration;
  if (std::isfinite(cycle_period) && cycle_period > duration) {
    sq_integral += (rest_torque * rest_torque) * (cycle_period - duration);
    period = cycle_period;
  }
  return std::sqrt(sq_integral / period);
}

TrajectoryValidator::TrajectoryValidator(
  ValidatorConfig config,
  std::shared_ptr<const StateValidator> state_validator,
  std::shared_ptr<const RneaDynamicsModel> dynamics)
: config_(std::move(config)),
  state_validator_(std::move(state_validator)),
  dynamics_(std::move(dynamics))
{
  if (!state_validator_) {
    throw std::invalid_argument("TrajectoryValidator requires a non-null StateValidator");
  }
  if (!std::isfinite(config_.validation_dt) || config_.validation_dt <= 0.0) {
    throw std::invalid_argument("validation_dt must be > 0");
  }
  if (config_.max_velocity.size() != kDof ||
    config_.max_acceleration.size() != kDof ||
    config_.max_jerk.size() != kDof)
  {
    throw std::invalid_argument("ValidatorConfig velocity/acceleration/jerk must have 4 elements");
  }
}

ValidationReport TrajectoryValidator::validate(
  const TimedTrajectory & trajectory,
  const State & expected_start,
  const State & expected_goal) const
{
  ValidationReport report;
  if (trajectory.points.size() < 2 || !isFiniteVec(expected_start) || !isFiniteVec(expected_goal)) {
    report.failure = FailureCode::kInvalidInput;
    report.message = "trajectory has fewer than 2 points or expected endpoints are invalid";
    return report;
  }

  if (config_.require_dynamics) {
    if (!dynamics_) {
      report.failure = FailureCode::kMissingDynamics;
      report.message = "dynamics model is missing in TrajectoryValidator";
      return report;
    }
    std::string reason;
    if (!dynamics_->config().hasValidDynamics(&reason)) {
      report.failure = FailureCode::kMissingDynamics;
      report.message = "dynamics model is incomplete: " + reason;
      return report;
    }
  }

  const auto & pts = trajectory.points;
  const std::size_t n = pts.size();

  // 1. 有限値と時刻の厳密な単調増加の検査（仕様書 第12.2節 1, 2項）
  if (!std::isfinite(pts.front().time) || std::abs(pts.front().time) > 1e-6) {
    report.failure = FailureCode::kNumericalError;
    report.message = "initial trajectory timestamp must be 0.0 and finite";
    return report;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = pts[i];
    if (!std::isfinite(p.time) || !isFiniteVec(p.position) || !isFiniteVec(p.velocity) ||
      !isFiniteVec(p.acceleration) || !isFiniteVec(p.jerk))
    {
      report.failure = FailureCode::kNumericalError;
      report.message = "non-finite kinematic value at point " + std::to_string(i);
      return report;
    }
    if (!p.torque.empty() && !isFiniteVec(p.torque)) {
      report.failure = FailureCode::kNumericalError;
      report.message = "non-finite torque value at point " + std::to_string(i);
      return report;
    }
    if (i > 0 && p.time <= pts[i - 1].time) {
      report.failure = FailureCode::kNumericalError;
      report.message = "timestamps are not strictly increasing at point " + std::to_string(i);
      return report;
    }
  }
  report.duration = pts.back().time - pts.front().time;

  // 7. 始点・終点、および指定された始終端速度・加速度との一致（仕様書 第12.2節 7項）
  const State zero(kDof, 0.0);
  const State & exp_v0 = config_.start_velocity_zero ? zero : config_.expected_start_velocity;
  const State & exp_vf = config_.goal_velocity_zero ? zero : config_.expected_goal_velocity;
  const State & exp_a0 = config_.start_acceleration_zero ? zero : config_.expected_start_acceleration;
  const State & exp_af = config_.goal_acceleration_zero ? zero : config_.expected_goal_acceleration;

  if (maxAbsDiff(pts.front().position, expected_start) > config_.endpoint_tolerance ||
    maxAbsDiff(pts.back().position, expected_goal) > config_.endpoint_tolerance)
  {
    report.failure = FailureCode::kJointLimitViolation;
    report.message = "trajectory start or goal position does not match expected endpoints";
    return report;
  }
  if (maxAbsDiff(pts.front().velocity, exp_v0) > config_.endpoint_tolerance ||
    maxAbsDiff(pts.back().velocity, exp_vf) > config_.endpoint_tolerance ||
    maxAbsDiff(pts.front().acceleration, exp_a0) > config_.endpoint_tolerance ||
    maxAbsDiff(pts.back().acceleration, exp_af) > config_.endpoint_tolerance)
  {
    report.failure = FailureCode::kJointLimitViolation;
    report.message = "trajectory boundary velocity or acceleration violates specification";
    return report;
  }

  // 3. 全軸の位置・速度・加速度・jerk 上限の検査（仕様書 第12.2節 3項）
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = pts[i];
    if (!config_.bounds.contains(p.position, kTol)) {
      report.failure = FailureCode::kJointLimitViolation;
      report.message = "joint position limit violated at t=" + std::to_string(p.time);
      return report;
    }
    for (std::size_t j = 0; j < kDof; ++j) {
      if (std::abs(p.velocity[j]) > config_.max_velocity[j] + kTol) {
        report.failure = FailureCode::kJointLimitViolation;
        report.message = "joint velocity limit violated on joint " + std::to_string(j) +
          " at t=" + std::to_string(p.time);
        return report;
      }
      if (std::abs(p.acceleration[j]) > config_.max_acceleration[j] + kTol) {
        report.failure = FailureCode::kJointLimitViolation;
        report.message = "joint acceleration limit violated on joint " + std::to_string(j) +
          " at t=" + std::to_string(p.time);
        return report;
      }
      if (std::abs(p.jerk[j]) > config_.max_jerk[j] + kTol) {
        report.failure = FailureCode::kJointLimitViolation;
        report.message = "joint jerk limit violated on joint " + std::to_string(j) +
          " at t=" + std::to_string(p.time);
        return report;
      }
    }
    if (i + 1 < n) {
      const double dt = pts[i + 1].time - p.time;
      for (std::size_t j = 0; j < kDof; ++j) {
        const double diff_jerk = (pts[i + 1].acceleration[j] - p.acceleration[j]) / dt;
        if (std::abs(diff_jerk) > config_.max_jerk[j] + 1e-4) {
          report.failure = FailureCode::kJointLimitViolation;
          report.message = "discrete acceleration difference violates jerk limit on joint " +
            std::to_string(j) + " at t=" + std::to_string(p.time);
          return report;
        }
      }
    }
  }

  // 4 & 5. validation_dt 以下の時間間隔で衝突および逆動力学トルクを検証する（仕様書 第12.2節 4, 5項）
  report.minimum_torque_margin = std::numeric_limits<double>::infinity();
  TimedTrajectory evaluated_traj;
  evaluated_traj.joint_names = trajectory.joint_names;
  evaluated_traj.points.reserve(n);

  const auto check_torque_at = [&](const DynamicState & state, State & out_tau) -> bool {
      if (!dynamics_) {
        out_tau = state.torque.size() == kDof ? state.torque : State(kDof, 0.0);
        return true;
      }
      out_tau = dynamics_->inverseDynamics(state.position, state.velocity, state.acceleration);
      // 格納されたトルクがある場合は、格納値と再計算値の厳しい方も検査する。
      for (std::size_t j = 0; j < kDof; ++j) {
        if (j != 0 && dynamics_->config().joint_peak_torque[j] <= 0.0) {
          continue;
        }
        const TorqueBounds tb = dynamics_->torqueBounds(j, state.velocity[j]);
        const double tau_check = (state.torque.size() == kDof &&
          std::abs(state.torque[j]) > std::abs(out_tau[j])) ? state.torque[j] : out_tau[j];
        if (!tb.feasible) {
          report.failure = FailureCode::kTorqueLimitViolation;
          report.violated_joint = j;
          report.violated_time = state.time;
          report.violated_position = state.position;
          report.required_torque = tau_check;
          report.allowed_torque = 0.0;
          report.violated_velocity = state.velocity[j];
          std::ostringstream oss;
          oss << "joint=" << j << " speed=" << state.velocity[j]
              << " rad/s is outside speed-torque curve range at t=" << state.time << "s";
          report.message = oss.str();
          return false;
        }
        const double margin = std::min(tb.max_torque - tau_check, tau_check - tb.min_torque);
        if (j == 0) {
          report.theta1_peak_torque = std::max(report.theta1_peak_torque, std::abs(tau_check));
          report.minimum_torque_margin = std::min(report.minimum_torque_margin, margin);
        }
        if (margin < -kTol) {
          report.failure = FailureCode::kTorqueLimitViolation;
          report.violated_joint = j;
          report.violated_time = state.time;
          report.violated_position = state.position;
          report.required_torque = tau_check;
          report.allowed_torque = tau_check >= 0.0 ? tb.max_torque : tb.min_torque;
          report.violated_velocity = state.velocity[j];
          std::ostringstream oss;
          oss << "torque limit violated on joint=" << j << " at t=" << state.time
              << "s: required=" << tau_check << " Nm, allowed=[" << tb.min_torque << ", "
              << tb.max_torque << "] Nm, velocity=" << state.velocity[j]
              << " rad/s, q=[" << state.position[0] << ", " << state.position[1] << ", "
              << state.position[2] << ", " << state.position[3] << "]";
          report.message = oss.str();
          return false;
        }
      }
      return true;
    };

  if (!state_validator_->isStateValid(pts.front().position)) {
    report.failure = FailureCode::kCollisionDetected;
    report.message = "collision detected at trajectory start";
    return report;
  }

  for (std::size_t i = 0; i + 1 < n; ++i) {
    const DynamicState & a = pts[i];
    const DynamicState & b = pts[i + 1];
    if (!state_validator_->isMotionValid(a.position, b.position)) {
      report.failure = FailureCode::kCollisionDetected;
      report.message = "collision detected between t=" + std::to_string(a.time) +
        "s and t=" + std::to_string(b.time) + "s";
      return report;
    }

    const double dt = b.time - a.time;
    const std::size_t sub = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(dt / config_.validation_dt)));
    for (std::size_t m = 0; m < sub; ++m) {
      const double r = static_cast<double>(m) / static_cast<double>(sub);
      DynamicState interp;
      interp.time = a.time + r * dt;
      interp.position.resize(kDof);
      interp.velocity.resize(kDof);
      interp.acceleration.resize(kDof);
      interp.jerk = a.jerk;
      if (a.torque.size() == kDof && b.torque.size() == kDof) {
        interp.torque.resize(kDof);
      }
      for (std::size_t j = 0; j < kDof; ++j) {
        interp.position[j] = a.position[j] + r * (b.position[j] - a.position[j]);
        interp.velocity[j] = a.velocity[j] + r * (b.velocity[j] - a.velocity[j]);
        interp.acceleration[j] = a.acceleration[j] + r * (b.acceleration[j] - a.acceleration[j]);
        if (!interp.torque.empty()) {
          interp.torque[j] = a.torque[j] + r * (b.torque[j] - a.torque[j]);
        }
      }
      if (!state_validator_->isStateValid(interp.position)) {
        report.failure = FailureCode::kCollisionDetected;
        report.message = "collision detected at interpolated time t=" + std::to_string(interp.time);
        return report;
      }
      State tau_eval;
      if (!check_torque_at(interp, tau_eval)) {
        return report;
      }
      if (dynamics_) {
        interp.torque = (interp.torque.size() == kDof &&
          std::abs(interp.torque[0]) > std::abs(tau_eval[0])) ? interp.torque : tau_eval;
      }
      evaluated_traj.points.push_back(std::move(interp));
    }
  }
  {
    DynamicState last = pts.back();
    if (!state_validator_->isStateValid(last.position)) {
      report.failure = FailureCode::kCollisionDetected;
      report.message = "collision detected at trajectory end";
      return report;
    }
    State tau_eval;
    if (!check_torque_at(last, tau_eval)) {
      return report;
    }
    if (dynamics_) {
      last.torque = (last.torque.size() == kDof &&
        std::abs(last.torque[0]) > std::abs(tau_eval[0])) ? last.torque : tau_eval;
    }
    evaluated_traj.points.push_back(std::move(last));
  }

  if (!std::isfinite(report.minimum_torque_margin)) {
    report.minimum_torque_margin = 0.0;
  }

  // 6. 第1軸 RMS トルクが連続定格上限内にあるか検査（仕様書 第12.2節 6項）
  report.theta1_rms_torque = computeTheta1RmsTorque(evaluated_traj, 0.0, 0.0);
  double rest_hold_torque = 0.0;
  if (dynamics_) {
    rest_hold_torque = dynamics_->gravityTorque(pts.back().position)[0];
  }
  report.theta1_cycle_rms_torque = computeTheta1RmsTorque(
    evaluated_traj, config_.cycle_period, rest_hold_torque);

  if (dynamics_) {
    const double cont_allowed = dynamics_->theta1ContinuousAllowedTorque();
    const double effective_rms =
      (config_.cycle_period > report.duration) ?
      std::max(report.theta1_rms_torque, report.theta1_cycle_rms_torque) :
      report.theta1_rms_torque;
    report.theta1_rms_margin = cont_allowed - effective_rms;
    if (cont_allowed > 0.0 && effective_rms > cont_allowed + kTol) {
      report.failure = FailureCode::kThermalLimitViolation;
      std::ostringstream oss;
      oss << "theta1 RMS torque " << effective_rms
          << " Nm exceeds continuous allowed torque " << cont_allowed << " Nm";
      report.message = oss.str();
      return report;
    }
  }

  return report;
}

}  // namespace path_planner
