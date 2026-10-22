#pragma once

// 弱トルク第1軸を考慮した経路・軌道生成パイプライン（仕様書 第1, 3.2, 6, 7.3, 9, 13.2, 14節）。

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "path_planner/collision_checker.hpp"
#include "path_planner/dynamics_model.hpp"
#include "path_planner/jerk_limiter.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/rrt_connect.hpp"
#include "path_planner/toppra_parameterizer.hpp"
#include "path_planner/torque_aware_cost.hpp"
#include "path_planner/trajectory_validator.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
class YamlNode;

enum class OperationMode
{
  kHardwareReady,
  kSimulationOnly,
  kValidationOnly
};

const char * toString(OperationMode mode);

struct TrajectoryPipelineConfig
{
  bool hardware_output_enabled{false};
  std::size_t candidate_count{8};
  std::uint32_t base_seed{42};
  double planning_time_limit{1.0};
  State distance_weights{2.0, 1.0, 1.0, 1.0};

  JointBounds joint_bounds{{-kPi, -kPi, -kPi, -kPi}, {kPi, kPi, kPi, kPi}};
  double rrt_step_size{0.12};
  double rrt_goal_bias{0.10};
  std::size_t rrt_max_iterations{3000};

  std::size_t shortcut_iterations{100};
  double max_path_deviation{0.01};
  double collision_check_resolution{0.01};
  double collision_margin{0.005};

  CostWeights cost_weights{};
  CostNormalization cost_normalization{};

  std::size_t topp_grid_points{200};
  std::size_t adaptive_grid_max_points{2000};
  double validation_dt{0.001};
  double cycle_period{0.0};
  bool start_velocity_zero{true};
  bool goal_velocity_zero{true};

  State max_velocity{0.0, 0.0, 0.0, 0.0};
  State max_acceleration{0.0, 0.0, 0.0, 0.0};
  State max_jerk{0.0, 0.0, 0.0, 0.0};

  bool enforce_torque_in_toppra{true};
  bool enforce_torque_in_jerk_limiter{true};
  double start_state_tolerance{0.02};

  std::vector<std::string> joint_names{"theta1", "theta2", "theta3", "theta4"};
  RobotCollisionModel collision_model;

  bool hasValidKinematicLimits(std::string * reason = nullptr) const;
};

/// Default URDF 衝突形状（upper_arm, forearm, wrist, flange）。
RobotCollisionModel defaultRobotCollisionModel(const KinematicChain & chain = defaultKinematicChain());

/// YAML の `trajectory_pipeline` ノードから設定を読み込む。
TrajectoryPipelineConfig loadTrajectoryPipelineConfig(
  const YamlNode & root,
  const KinematicChain & chain = defaultKinematicChain());

class TrajectoryPipeline
{
public:
  /// hardware_output_enabled=true かつ必須パラメータ未設定なら std::runtime_error を送出する。
  TrajectoryPipeline(
    TrajectoryPipelineConfig pipeline_config,
    RobotDynamicsConfig dynamics_config,
    std::shared_ptr<const CollisionScene> initial_scene = nullptr);

  OperationMode operationMode() const {return mode_;}
  bool isHardwareOutputAllowed() const
  {
    return config_.hardware_output_enabled && mode_ == OperationMode::kHardwareReady;
  }

  /// 始終点から計画・後処理・候補選択・TOPP-RA・jerk制限・最終検証を一括実行する。
  /// cancel_requested が途中で true になった場合、または計画中にシーン・ペイロードが
  /// 変化した場合は結果を破棄して失敗を返す。
  PipelineResult execute(
    const State & start,
    const State & goal,
    std::uint32_t seed = 0,
    const std::atomic<bool> * cancel_requested = nullptr) const;

  /// 与えられた候補折れ線群に対して後処理・コスト評価・選択・時間化・最終検証を行う。
  PipelineResult executeFromCandidates(
    const std::vector<Path> & candidate_paths,
    const State & start,
    const State & goal,
    std::uint32_t seed = 42) const;

  /// ペイロードを更新し、動力学モデルのバージョンを進める（以前の軌道は無効化される）。
  void updatePayload(const Payload & payload);

  /// 障害物シーンを更新し、シーンのバージョンを進める（計画中の軌道は無効化される）。
  void updateCollisionScene(std::vector<Obstacle> obstacles);

  /// 実行直前に、生成された軌道が現在のペイロード/シーン版および現在姿勢と整合するか検査する。
  bool isTrajectoryValidForExecution(
    const PipelineResult & result,
    const State & current_position,
    std::string * reason = nullptr) const;

  std::uint64_t currentDynamicsVersion() const;
  std::uint64_t currentSceneVersion() const;
  const TrajectoryPipelineConfig & config() const {return config_;}
  RobotDynamicsConfig dynamicsConfig() const;
  std::shared_ptr<const CollisionScene> collisionScene() const;

private:
  TrajectoryPipelineConfig config_;
  OperationMode mode_{OperationMode::kValidationOnly};

  mutable std::mutex mutex_;
  std::shared_ptr<const RneaDynamicsModel> dynamics_;
  std::shared_ptr<const CollisionScene> scene_;

  PipelineResult parameterizeAndValidate(
    const std::vector<ParameterizedPath> & splines,
    const std::vector<Path> & raw_candidates,
    const State & start,
    const State & goal,
    const std::shared_ptr<const RneaDynamicsModel> & dyn_snapshot,
    const std::shared_ptr<const CollisionScene> & scene_snapshot,
    const std::shared_ptr<const StateValidator> & state_validator,
    const std::atomic<bool> * cancel_requested,
    std::vector< std::pair<std::string, double>> stage_durations) const;
};

}  // namespace path_planner
