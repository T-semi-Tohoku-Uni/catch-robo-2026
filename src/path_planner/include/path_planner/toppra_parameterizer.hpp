#pragma once

// 逆動力学制約付き TOPP-RA 時間パラメータ化（仕様書 第11節）。
//
// - 汎用 LP/QP ソルバを使用せず、一次不等式の解析的な交差と速度―トルク曲線の保守的区分線形反復で解く。
// - 経路曲率・動力学係数の変化が大きい区間を自動細分化する適応格子を備える。
// - 解が存在しない場合（静的重力トルク超過など）は制約を緩和せず明示的に失敗を返す。

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "path_planner/dynamics_model.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
struct ToppraConfig
{
  std::size_t grid_points{200};
  std::size_t adaptive_grid_max_points{2000};
  bool start_velocity_zero{true};
  bool goal_velocity_zero{true};
  double start_s_dot{0.0};
  double goal_s_dot{0.0};

  State max_velocity{1.0, 1.0, 1.0, 1.0};
  State max_acceleration{1.0, 1.0, 1.0, 1.0};

  /// true のとき RneaDynamicsModel による第1軸（および設定された他軸）トルク制約を課す。
  bool enable_torque_constraints{true};
  std::size_t max_curve_iterations{8};
  double curve_convergence_tol{1e-4};
};

/// TOPP-RA 格子上の時間プロファイル（jerk 制限器への入力および検証用）。
struct ToppraGridProfile
{
  std::vector<double> s;        // 格子位置 s_i in [0, 1]
  std::vector<double> s_dot;    // 経路速度 ds/dt >= 0
  std::vector<double> s_ddot;   // 区間経路加速度 d2s/dt2（最終点は前区間値）
  std::vector<double> time;     // 時刻 t_i [s]
};

struct ToppraResult
{
  ToppraGridProfile profile;
  TimedTrajectory trajectory;
  FailureCode failure{FailureCode::kNone};
  std::string message;
  double duration{0.0};
  double theta1_peak_torque{0.0};
  double theta1_rms_torque{0.0};
  double minimum_torque_margin{0.0};
  double minimum_velocity_margin{0.0};
  double minimum_acceleration_margin{0.0};
  std::string bottleneck;  // 例: "joint=0 type=torque time=0.42s"

  explicit operator bool() const {return failure == FailureCode::kNone && !trajectory.points.empty();}
};

class ToppraParameterizer
{
public:
  ToppraParameterizer(
    ToppraConfig config,
    std::shared_ptr<const RneaDynamicsModel> dynamics = nullptr,
    std::vector<std::string> joint_names = {"theta1", "theta2", "theta3", "theta4"});

  /// 適応格子を構築する（単体テストからも直接検証可能）。
  std::vector<double> buildAdaptiveGrid(const ParameterizedPath & path) const;

  /// ParameterizedPath を時間パラメータ化する。
  /// velocity_scale in (0, 1] は jerk 制限器からの再時間化時に全体を遅くするために用いる。
  ToppraResult parameterize(
    const ParameterizedPath & path,
    double velocity_scale = 1.0) const;

  const ToppraConfig & config() const {return config_;}

private:
  ToppraConfig config_;
  std::shared_ptr<const RneaDynamicsModel> dynamics_;
  std::vector<std::string> joint_names_;
};

}  // namespace path_planner
