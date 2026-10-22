#pragma once

// 経路・軌道生成パイプライン共通のデータ型（仕様書 第6節）。
// 状態順序は全モジュールで [theta1, theta2, theta3, theta4]、単位は SI（rad, s, N m）。

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace path_planner
{
constexpr std::size_t kDof = 4;
constexpr double kPi = 3.14159265358979323846;

using State = std::vector<double>;
using Path = std::vector<State>;

struct DynamicState
{
  State position;
  State velocity;
  State acceleration;
  State jerk;
  State torque;
  double time{0.0};
};

struct TimedTrajectory
{
  std::vector<std::string> joint_names;
  std::vector<DynamicState> points;
};

enum class FailureCode : std::uint16_t
{
  kNone = 0,
  kInvalidInput,
  kMissingDynamics,
  kPlanningFailed,
  kSmoothingFailed,
  kTimeParameterizationInfeasible,
  kJerkLimitInfeasible,
  kCollisionDetected,
  kJointLimitViolation,
  kTorqueLimitViolation,
  kThermalLimitViolation,
  kNumericalError
};

const char * toString(FailureCode code);

struct PipelineResult
{
  TimedTrajectory trajectory;
  FailureCode failure{FailureCode::kNone};
  std::string message;
  double duration{0.0};
  double theta1_peak_torque{0.0};
  double theta1_rms_torque{0.0};
  double minimum_torque_margin{0.0};
  std::size_t selected_candidate{0};

  // ---- 追加診断値（仕様書 第11.3, 13, 15, 17節） ----
  bool torque_verified{false};          // 動力学モデルでトルク検証済みか
  double theta1_rms_margin{0.0};        // 連続定格 - RMS (N m)
  std::string failure_stage;            // 失敗した処理段階
  std::string bottleneck;               // TOPP-RA のボトルネック（軸・制約種別・時刻）
  Path geometric_path;                  // 選択候補の未時間化経路（デバッグ用）
  std::vector<Path> candidate_paths;    // 全候補の短絡化後経路（可視化・比較用）
  std::size_t candidate_count{0};       // 得られた有効候補数
  std::uint64_t dynamics_version{0};    // 計画時に使用した動力学モデル版
  std::uint64_t scene_version{0};       // 計画時に使用した衝突シーン版
  std::vector<std::pair<std::string, double>> stage_durations;  // 各処理時間 (s)

  explicit operator bool() const {return failure == FailureCode::kNone;}
};

/// 4要素で全要素が有限値か。
bool isFiniteState(const State & q);

}  // namespace path_planner
