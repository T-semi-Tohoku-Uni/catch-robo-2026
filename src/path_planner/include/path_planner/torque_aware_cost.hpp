#pragma once

// 第1軸負荷を考慮した候補経路評価（仕様書 第9節）。
//
//   J = w_L (L / L_ref)
//     + w_1 (int_0^1 |q'_1(s)| ds / theta1_ref)
//     + w_I (int_0^1 I_{1,eff}(q(s)) ds / I_ref)
//     + w_tau (int_0^1 (tau1_hat(s) / tau_ref)^2 ds)
//     + w_C C_clearance

#include <cstddef>
#include <memory>
#include <vector>

#include "path_planner/collision_checker.hpp"
#include "path_planner/dynamics_model.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
struct CostWeights
{
  double path_length{1.0};
  double theta1_motion{2.0};
  double theta1_inertia{2.0};
  double theta1_torque{4.0};
  double clearance{1.0};
};

struct CostNormalization
{
  double path_length_ref{1.0};
  double theta1_motion_ref{1.0};
  double theta1_inertia_ref{0.2};
  double theta1_torque_ref{5.0};
  double clearance_ref{0.05};
  double test_s_dot{1.0};
  double test_s_ddot{2.0};
  std::size_t sample_count{64};
};

struct CandidateScore
{
  double total_cost{0.0};
  double weighted_path_length{0.0};
  double theta1_variation{0.0};
  double mean_theta1_inertia{0.0};
  double mean_sq_theta1_torque{0.0};
  double clearance_penalty{0.0};
  double min_clearance{0.0};
};

class TorqueAwareCost
{
public:
  TorqueAwareCost(
    CostWeights weights,
    State distance_weights,
    std::shared_ptr<const DynamicsModel> dynamics,
    std::shared_ptr<const StateValidator> validator,
    CostNormalization normalization = {});

  CandidateScore evaluate(const ParameterizedPath & path) const;

  /// 複数候補の中から最小スコアの添字を返す（空なら 0）。
  std::size_t selectBest(
    const std::vector<ParameterizedPath> & candidates,
    std::vector<CandidateScore> * scores_out = nullptr) const;

  const CostWeights & weights() const {return weights_;}

private:
  CostWeights weights_;
  State distance_weights_;
  std::shared_ptr<const DynamicsModel> dynamics_;
  std::shared_ptr<const StateValidator> validator_;
  CostNormalization norm_;
};

}  // namespace path_planner
