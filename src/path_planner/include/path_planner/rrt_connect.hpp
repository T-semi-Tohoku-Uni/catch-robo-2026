#pragma once

// RRT-Connect 幾何経路探索（仕様書 第7節）。
//
// 要件:
// - 始点木と終点木を交互に拡張する。
// - EXTEND 成功後は同じ目標へ TRAPPED または REACHED まで反復して伸ばす（CONNECT）。
// - lower_bounds / upper_bounds を使用する。
// - 最近傍距離は関節ごとの重み付き距離とし、distance_weights[0] >= 1 を設定可能にする。
// - ノードだけでなく全辺に StateValidator を適用する。
// - 始点・終点が無効なら探索を開始しない。
// - seed 固定時は同じ経路を返す。

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "path_planner/collision_checker.hpp"
#include "path_planner/geometric_planner.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
struct RrtConnectConfig
{
  std::size_t active_axes{4};
  JointBounds bounds{{-kPi, -kPi, -kPi, -kPi}, {kPi, kPi, kPi, kPi}};
  State distance_weights{2.0, 1.0, 1.0, 1.0};
  double step_size{0.10};
  double goal_bias{0.10};
  std::size_t max_iterations{3000};
};

class RrtConnect : public GeometricPlanner
{
public:
  RrtConnect(RrtConnectConfig config, std::shared_ptr<const StateValidator> validator);

  PlanResult plan(
    const State & start, const State & goal,
    std::uint32_t seed) const override;

  double weightedDistance(const State & a, const State & b) const;
  const RrtConnectConfig & config() const {return config_;}
  const std::shared_ptr<const StateValidator> & validator() const {return validator_;}

private:
  RrtConnectConfig config_;
  std::shared_ptr<const StateValidator> validator_;
};

// ============================================================================
// 既存試作 API（後方互換用）
// ============================================================================
struct ArmLinkDimensions
{
  double upper_arm_length{0.480};
  double forearm_length{0.480};
  std::array<double, 3> upper_arm_size{0.191, 0.071, 0.480};
  std::array<double, 3> forearm_size{0.07025, 0.071, 0.480};
  std::array<double, 3> wrist_size{0.040, 0.050, 0.050};
  std::array<double, 3> hand_size{0.027, 0.492, 0.124};
};

struct PlannerOptions
{
  double goal_radius{0.01};
  double step_size{0.02};
  double collision_resolution{0.02};
  std::uint32_t max_iterations{5000};
  ArmLinkDimensions arm_dimensions{};
};

class RRT_connect
{
public:
  explicit RRT_connect(PlannerOptions options = {})
  : options_(options) {}

  bool isPathPlanning(
    const std::array<double, 4> & start,
    const std::array<double, 4> & end,
    std::vector<std::array<double, 4>> & path);

private:
  PlannerOptions options_;
  bool isStateCollision(const std::array<double, 4> & state);
  bool isObstacleDetection(
    const std::array<double, 4> & start,
    const std::array<double, 4> & end);
};

}  // namespace path_planner
