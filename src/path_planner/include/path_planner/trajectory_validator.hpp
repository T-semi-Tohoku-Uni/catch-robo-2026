#pragma once

// 時間付き軌道の最終検証器（仕様書 第12.2, 15節）。
//
// 計画器の成功は実行可能性の保証ではなく、この最終検証器だけを実行許可の判定点とする。

#include <cstddef>
#include <memory>
#include <string>

#include "path_planner/collision_checker.hpp"
#include "path_planner/dynamics_model.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
struct ValidatorConfig
{
  JointBounds bounds{{-kPi, -kPi, -kPi, -kPi}, {kPi, kPi, kPi, kPi}};
  State max_velocity{1.0, 1.0, 1.0, 1.0};
  State max_acceleration{1.0, 1.0, 1.0, 1.0};
  State max_jerk{10.0, 10.0, 10.0, 10.0};

  double validation_dt{0.001};
  double cycle_period{0.0};  // > duration のとき休止時間込み周期で RMS トルクを計算する

  bool start_velocity_zero{true};
  bool goal_velocity_zero{true};
  bool start_acceleration_zero{true};
  bool goal_acceleration_zero{true};
  State expected_start_velocity{0.0, 0.0, 0.0, 0.0};
  State expected_goal_velocity{0.0, 0.0, 0.0, 0.0};
  State expected_start_acceleration{0.0, 0.0, 0.0, 0.0};
  State expected_goal_acceleration{0.0, 0.0, 0.0, 0.0};

  double endpoint_tolerance{1e-4};
  bool require_dynamics{true};
};

struct ValidationReport
{
  FailureCode failure{FailureCode::kNone};
  std::string message;
  double duration{0.0};
  double theta1_peak_torque{0.0};
  double theta1_rms_torque{0.0};
  double theta1_cycle_rms_torque{0.0};
  double minimum_torque_margin{0.0};
  double theta1_rms_margin{0.0};

  // トルク違反詳細（仕様書 第15節）
  std::size_t violated_joint{0};
  double violated_time{0.0};
  State violated_position;
  double required_torque{0.0};
  double allowed_torque{0.0};
  double violated_velocity{0.0};

  explicit operator bool() const {return failure == FailureCode::kNone;}
};

/// 第1軸 RMS トルクを計算する。cycle_period > T のときは休止区間（トルク 0 または保持トルク）を考慮した
/// 周期全体の RMS を返す。rest_torque は休止中の保持トルク [N m]。
double computeTheta1RmsTorque(
  const TimedTrajectory & trajectory,
  double cycle_period = 0.0,
  double rest_torque = 0.0);

class TrajectoryValidator
{
public:
  TrajectoryValidator(
    ValidatorConfig config,
    std::shared_ptr<const StateValidator> state_validator,
    std::shared_ptr<const RneaDynamicsModel> dynamics = nullptr);

  ValidationReport validate(
    const TimedTrajectory & trajectory,
    const State & expected_start,
    const State & expected_goal) const;

  const ValidatorConfig & config() const {return config_;}

private:
  ValidatorConfig config_;
  std::shared_ptr<const StateValidator> state_validator_;
  std::shared_ptr<const RneaDynamicsModel> dynamics_;
};

}  // namespace path_planner
