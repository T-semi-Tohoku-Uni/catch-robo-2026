#pragma once

// ROS 2 アダプターおよび軌道生成ノード（仕様書 第13, 15節）。
//
// - /geometric_path (trajectory_msgs/msg/JointTrajectory): デバッグ用未時間化経路
// - /timed_trajectory (trajectory_msgs/msg/JointTrajectory): 最終検証に成功した場合のみ配信
// - /trajectory_diagnostics (diagnostic_msgs/msg/DiagnosticArray): 余裕、ピーク/RMS、失敗理由
// - /generate_trajectory (service): 始終点・ペイロード・seed から計画・時間化を一括実行
// - 同一ノードで同時に複数の計画を実行せず、reject または cancel_previous を設定可能にする。
// - ROS 2 が無いビルド環境でも同一ロジックをテスト・実行できるよう共通メッセージ構造体を備える。

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "path_planner/trajectory_follower.hpp"
#include "path_planner/trajectory_pipeline.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
enum class ConcurrencyPolicy
{
  kReject,
  kCancelPrevious
};

/// trajectory_msgs/msg/JointTrajectoryPoint 相当のメッセージ表現。
struct JointTrajectoryPointMsg
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> effort;
  double time_from_start_sec{0.0};
};

/// trajectory_msgs/msg/JointTrajectory 相当のメッセージ表現。
struct JointTrajectoryMsg
{
  std::string request_id;
  double generated_timestamp_sec{0.0};
  std::string frame_id{"map"};
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPointMsg> points;
};

/// diagnostic_msgs/msg/DiagnosticStatus + DiagnosticArray 相当のメッセージ表現。
struct DiagnosticArrayMsg
{
  enum class Level : std::uint8_t {kOk = 0, kWarn = 1, kError = 2};

  std::string request_id;
  double timestamp_sec{0.0};
  Level level{Level::kOk};
  std::string name{"trajectory_pipeline"};
  std::string message;
  std::map<std::string, std::string> key_values;
};

/// /generate_trajectory サービス要求（仕様書 第13.1節）。
struct GenerateTrajectoryRequest
{
  std::vector<std::string> start_joint_names;
  State start_positions;
  std::vector<std::string> goal_joint_names;
  State goal_positions;
  bool has_payload{false};
  Payload payload{};
  std::uint32_t seed{0};
  bool has_current_joint_state{false};
  State current_positions;
};

/// /generate_trajectory サービス応答（仕様書 第13.1節）。
struct GenerateTrajectoryResponse
{
  std::string request_id;
  bool success{false};
  std::uint16_t failure_code{0};
  std::string message;
  JointTrajectoryMsg trajectory;
  double theta1_peak_torque{0.0};
  double theta1_rms_torque{0.0};
  double minimum_torque_margin{0.0};
};

/// ROS-independent timed-follow request.
struct FollowTrajectoryGoalRequest
{
  bool start{true};
  bool plan_from_start_goal{false};
  std::vector<std::string> start_joint_names;
  State start_positions;
  std::vector<std::string> goal_joint_names;
  State goal_positions;
  bool has_payload{false};
  Payload payload{};
  std::uint32_t seed{0};
  JointTrajectoryMsg trajectory;
};

/// ROS-independent timed-follow result.
struct FollowTrajectoryActionResult
{
  bool success{false};
  std::uint16_t failure_code{0};
  std::string message;
  double elapsed_time_sec{0.0};
  double final_position_error_mm{0.0};
  double final_yaw_error_rad{0.0};
  double max_joint_tracking_error_rad{0.0};
};

class TrajectoryNode
{
public:
  TrajectoryNode(
    std::shared_ptr<TrajectoryPipeline> pipeline,
    ConcurrencyPolicy policy = ConcurrencyPolicy::kReject,
    TrajectoryFollowerConfig follower_config = TrajectoryFollowerConfig{});

  /// /generate_trajectory サービス要求を処理する。
  /// 成功時のみ lastTimedTrajectory() を更新し、失敗時は直前の成功軌道を再利用させない。
  GenerateTrajectoryResponse handleGenerateTrajectory(const GenerateTrajectoryRequest & request);

  /// Validate and start a timed trajectory, optionally planning it first.
  bool acceptFollowTrajectoryGoal(
    const FollowTrajectoryGoalRequest & goal,
    double now_sec,
    FollowTrajectoryActionResult * out_immediate_failure = nullptr);

  /// Convert a trajectory message into the internal four-axis form.
  FollowerStepOutput stepFollowTrajectory(double now_sec);

  bool convertMsgToTimedTrajectory(
    const JointTrajectoryMsg & msg,
    TimedTrajectory & out_traj,
    std::string * reason = nullptr) const;

  /// 現在進行中の計画へキャンセル要求を出す（cancel_previous のテスト・外部制御用）。
  void requestCancelCurrent();

  /// 直近の配信済みメッセージの取得（テストおよびアダプター連携用）。
  bool hasPublishedTimedTrajectory() const;
  JointTrajectoryMsg lastTimedTrajectory() const;
  JointTrajectoryMsg lastGeometricPath() const;
  DiagnosticArrayMsg lastDiagnostics() const;
  PipelineResult lastPipelineResult() const;
  std::uint64_t publishedTimedTrajectoryCount() const {return timed_publish_count_.load();}

  const std::shared_ptr<TrajectoryPipeline> & pipeline() const {return pipeline_;}
  const std::shared_ptr<TrajectoryFollower> & follower() const {return follower_;}

private:
  std::shared_ptr<TrajectoryPipeline> pipeline_;
  std::shared_ptr<TrajectoryFollower> follower_;
  ConcurrencyPolicy policy_;
  std::atomic<std::uint64_t> execution_dynamics_version_{0},execution_scene_version_{0};

  mutable std::mutex active_mutex_;
  bool planning_active_{false};
  std::atomic<bool> cancel_flag_{false};
  std::atomic<std::uint64_t> request_counter_{0};
  std::atomic<std::uint64_t> timed_publish_count_{0};

  mutable std::mutex pub_mutex_;
  bool has_timed_trajectory_{false};
  JointTrajectoryMsg last_timed_trajectory_;
  JointTrajectoryMsg last_geometric_path_;
  DiagnosticArrayMsg last_diagnostics_;
  PipelineResult last_pipeline_result_;

  bool reorderJointState(
    const std::vector<std::string> & names,
    const State & values,
    State & out_ordered) const;
};

}  // namespace path_planner
