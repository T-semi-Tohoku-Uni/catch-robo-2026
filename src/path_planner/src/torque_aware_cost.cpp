#include "path_planner/torque_aware_cost.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace path_planner
{
TorqueAwareCost::TorqueAwareCost(
  CostWeights weights,
  State distance_weights,
  std::shared_ptr<const DynamicsModel> dynamics,
  std::shared_ptr<const StateValidator> validator,
  CostNormalization normalization)
: weights_(weights),
  distance_weights_(std::move(distance_weights)),
  dynamics_(std::move(dynamics)),
  validator_(std::move(validator)),
  norm_(normalization)
{
  if (!std::isfinite(weights_.path_length) || weights_.path_length < 0.0 ||
    !std::isfinite(weights_.theta1_motion) || weights_.theta1_motion < 0.0 ||
    !std::isfinite(weights_.theta1_inertia) || weights_.theta1_inertia < 0.0 ||
    !std::isfinite(weights_.theta1_torque) || weights_.theta1_torque < 0.0 ||
    !std::isfinite(weights_.clearance) || weights_.clearance < 0.0)
  {
    throw std::invalid_argument("CostWeights must be finite and >= 0");
  }
  if (distance_weights_.size() != kDof) {
    throw std::invalid_argument("distance_weights must have 4 elements");
  }
  for (double w : distance_weights_) {
    if (!std::isfinite(w) || w < 0.0) {
      throw std::invalid_argument("distance_weights must be >= 0");
    }
  }
  if (norm_.path_length_ref <= 0.0 || norm_.theta1_motion_ref <= 0.0 ||
    norm_.theta1_inertia_ref <= 0.0 || norm_.theta1_torque_ref <= 0.0 ||
    norm_.clearance_ref <= 0.0 || norm_.sample_count < 2)
  {
    throw std::invalid_argument("CostNormalization reference values must be positive");
  }
}

CandidateScore TorqueAwareCost::evaluate(const ParameterizedPath & path) const
{
  if (path.empty()) {
    throw std::invalid_argument("cannot evaluate an empty ParameterizedPath");
  }
  CandidateScore score;
  score.min_clearance = std::numeric_limits<double>::infinity();

  const std::size_t n = norm_.sample_count;
  const double ds = 1.0 / static_cast<double>(n);
  for (std::size_t i = 0; i <= n; ++i) {
    const double s = static_cast<double>(i) * ds;
    const double trap_w = (i == 0 || i == n) ? (0.5 * ds) : ds;

    const State q = path.position(s);
    const State dq = path.firstDerivative(s);
    const State d2q = path.secondDerivative(s);

    // 重み付き経路長微分 ||W * q'(s)||
    double weighted_norm_sq = 0.0;
    for (std::size_t j = 0; j < kDof; ++j) {
      const double v = distance_weights_[j] * dq[j];
      weighted_norm_sq += v * v;
    }
    score.weighted_path_length += trap_w * std::sqrt(weighted_norm_sq);
    score.theta1_variation += trap_w * std::abs(dq[0]);

    if (dynamics_) {
      const double i1_eff = dynamics_->effectiveInertia(0, q);
      score.mean_theta1_inertia += trap_w * i1_eff;

      // 保守的な試験速度・正負の試験加速度での第1軸概算トルク（単位弧長接線方向で評価）
      double dq_norm = 0.0;
      for (std::size_t j = 0; j < kDof; ++j) {
        dq_norm += dq[j] * dq[j];
      }
      dq_norm = std::sqrt(dq_norm);
      const double inv_norm = dq_norm > 1e-9 ? (1.0 / dq_norm) : 0.0;
      const double inv_norm_sq = inv_norm * inv_norm;

      const double s_dot = norm_.test_s_dot;
      const double s_ddot = norm_.test_s_ddot;
      State qd(kDof);
      State qdd_pos(kDof);
      State qdd_neg(kDof);
      for (std::size_t j = 0; j < kDof; ++j) {
        const double tangent_j = dq[j] * inv_norm;
        const double curv_j = d2q[j] * inv_norm_sq;
        qd[j] = tangent_j * s_dot;
        qdd_pos[j] = tangent_j * s_ddot + curv_j * (s_dot * s_dot);
        qdd_neg[j] = -tangent_j * s_ddot + curv_j * (s_dot * s_dot);
      }
      const State tau_pos = dynamics_->inverseDynamics(q, qd, qdd_pos);
      const State tau_neg = dynamics_->inverseDynamics(q, qd, qdd_neg);
      const double tau1_hat = std::max(std::abs(tau_pos[0]), std::abs(tau_neg[0]));
      score.mean_sq_theta1_torque += trap_w * (tau1_hat * tau1_hat);
    }

    if (validator_) {
      const double c = validator_->clearance(q);
      score.min_clearance = std::min(score.min_clearance, c);
      if (std::isfinite(c)) {
        const double ratio = std::max(0.0, norm_.clearance_ref - c) / norm_.clearance_ref;
        score.clearance_penalty += trap_w * (ratio * ratio);
      }
    }
  }

  const double l_term = score.weighted_path_length / norm_.path_length_ref;
  const double m_term = score.theta1_variation / norm_.theta1_motion_ref;
  const double i_term = score.mean_theta1_inertia / norm_.theta1_inertia_ref;
  const double t_term =
    score.mean_sq_theta1_torque / (norm_.theta1_torque_ref * norm_.theta1_torque_ref);

  score.total_cost =
    weights_.path_length * l_term +
    weights_.theta1_motion * m_term +
    weights_.theta1_inertia * i_term +
    weights_.theta1_torque * t_term +
    weights_.clearance * score.clearance_penalty;
  return score;
}

std::size_t TorqueAwareCost::selectBest(
  const std::vector<ParameterizedPath> & candidates,
  std::vector<CandidateScore> * scores_out) const
{
  if (scores_out != nullptr) {
    scores_out->clear();
    scores_out->reserve(candidates.size());
  }
  if (candidates.empty()) {
    return 0;
  }
  std::size_t best_idx = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const CandidateScore s = evaluate(candidates[i]);
    if (scores_out != nullptr) {
      scores_out->push_back(s);
    }
    if (s.total_cost < best_cost) {
      best_cost = s.total_cost;
      best_idx = i;
    }
  }
  return best_idx;
}

}  // namespace path_planner
