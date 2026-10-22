#pragma once

// jerk 制限付き時間再パラメータ化（仕様書 第12.1節）。
//
// - 幾何経路 q(s) は一切変更せず、s(t) のみを滑らかに時間拡大する。
// - 離散時刻列の加速度差分から jerk を計算し、違反区間とその前後の時間幅を反復的に拡大する。
// - 各反復後に速度・加速度・jerk・逆動力学トルクを再計算し、制約違反が残る場合は
//   全体または該当区間を時間方向に拡大して再試行する。

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "path_planner/dynamics_model.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/toppra_parameterizer.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
struct JerkLimiterConfig
{
  State max_velocity{1.0, 1.0, 1.0, 1.0};
  State max_acceleration{1.0, 1.0, 1.0, 1.0};
  State max_jerk{10.0, 10.0, 10.0, 10.0};
  bool start_velocity_zero{true};
  bool goal_velocity_zero{true};
  bool start_acceleration_zero{true};
  bool goal_acceleration_zero{true};
  bool enforce_torque_limits{true};
  double sample_dt{0.001};
  std::size_t max_iterations{40};
};

struct JerkLimitResult
{
  TimedTrajectory trajectory;
  FailureCode failure{FailureCode::kNone};
  std::string message;
  double duration{0.0};
  double theta1_peak_torque{0.0};
  double theta1_rms_torque{0.0};
  double minimum_torque_margin{0.0};
  std::size_t iterations_used{0};

  explicit operator bool() const
  {
    return failure == FailureCode::kNone && !trajectory.points.empty();
  }
};

class JerkLimiter
{
public:
  JerkLimiter(
    JerkLimiterConfig config,
    std::shared_ptr<const RneaDynamicsModel> dynamics = nullptr,
    std::vector<std::string> joint_names = {"theta1", "theta2", "theta3", "theta4"});

  JerkLimitResult limit(
    const ParameterizedPath & path,
    const ToppraGridProfile & topp_profile) const;

  const JerkLimiterConfig & config() const {return config_;}

private:
  JerkLimiterConfig config_;
  std::shared_ptr<const RneaDynamicsModel> dynamics_;
  std::vector<std::string> joint_names_;
};

}  // namespace path_planner
