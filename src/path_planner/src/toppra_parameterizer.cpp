#include "path_planner/toppra_parameterizer.hpp"

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
constexpr double kEps = 1e-11;
constexpr double kConstraintSlack = 1e-9;

struct GridNodeData
{
  double s{0.0};
  State q;
  State dq;
  State d2q;
  State m;  // M(q) * q'(s)
  State b;  // M(q) * q''(s) + C(q, q'(s)) * q'(s)
  State g;  // g(q)
};

void intersectInterval(double lo, double hi, double & cur_lo, double & cur_hi)
{
  cur_lo = std::max(cur_lo, lo);
  cur_hi = std::min(cur_hi, hi);
}

// a * u in [lo, hi] を [u_lo, u_hi] に交差させる。
bool intersectLinear(double a, double lo, double hi, double & u_lo, double & u_hi)
{
  if (!std::isfinite(a) || !std::isfinite(lo) || !std::isfinite(hi) || lo > hi + kConstraintSlack) {
    return false;
  }
  if (std::abs(a) <= kEps) {
    return lo <= kConstraintSlack && hi >= -kConstraintSlack;
  }
  const double b1 = lo / a;
  const double b2 = hi / a;
  intersectInterval(std::min(b1, b2), std::max(b1, b2), u_lo, u_hi);
  return u_lo <= u_hi + kConstraintSlack;
}
}  // namespace

ToppraParameterizer::ToppraParameterizer(
  ToppraConfig config,
  std::shared_ptr<const RneaDynamicsModel> dynamics,
  std::vector<std::string> joint_names)
: config_(std::move(config)),
  dynamics_(std::move(dynamics)),
  joint_names_(std::move(joint_names))
{
  if (config_.grid_points < 2) {
    throw std::invalid_argument("topp_grid_points must be >= 2");
  }
  if (config_.adaptive_grid_max_points < config_.grid_points) {
    throw std::invalid_argument("adaptive_grid_max_points must be >= topp_grid_points");
  }
  if (config_.max_velocity.size() != kDof || config_.max_acceleration.size() != kDof) {
    throw std::invalid_argument("max_velocity and max_acceleration must have 4 elements");
  }
  for (std::size_t j = 0; j < kDof; ++j) {
    if (!std::isfinite(config_.max_velocity[j]) || config_.max_velocity[j] <= 0.0 ||
      !std::isfinite(config_.max_acceleration[j]) || config_.max_acceleration[j] <= 0.0)
    {
      throw std::invalid_argument("max_velocity and max_acceleration must be positive");
    }
  }
  if (config_.max_curve_iterations == 0) {
    throw std::invalid_argument("max_curve_iterations must be >= 1");
  }
}

std::vector<double> ToppraParameterizer::buildAdaptiveGrid(const ParameterizedPath & path) const
{
  const std::size_t base_n = config_.grid_points;
  std::vector<double> grid;
  grid.reserve(std::min<std::size_t>(config_.adaptive_grid_max_points, base_n * 2));
  for (std::size_t i = 0; i < base_n; ++i) {
    grid.push_back(static_cast<double>(i) / static_cast<double>(base_n - 1));
  }
  grid.front() = 0.0;
  grid.back() = 1.0;

  if (path.isStationary() || config_.adaptive_grid_max_points <= grid.size()) {
    return grid;
  }

  // 各区間の曲率変化 ||q''(s_{i+1}) - q''(s_i)|| と第1軸慣性変化を評価し、大きい区間を細分化する。
  std::vector<double> weights(grid.size() - 1, 0.0);
  double sum_w = 0.0;
  for (std::size_t i = 0; i + 1 < grid.size(); ++i) {
    const double s0 = grid[i];
    const double s1 = grid[i + 1];
    const double sm = 0.5 * (s0 + s1);
    const State d2q0 = path.secondDerivative(s0);
    const State d2q1 = path.secondDerivative(s1);
    const State d2qm = path.secondDerivative(sm);
    double curv = 0.0;
    for (std::size_t j = 0; j < kDof; ++j) {
      curv += std::abs(d2q1[j] - d2q0[j]) + 0.5 * std::abs(d2qm[j]);
    }
    if (dynamics_) {
      const double i0 = dynamics_->effectiveInertia(0, path.position(s0));
      const double i1 = dynamics_->effectiveInertia(0, path.position(s1));
      curv += 5.0 * std::abs(i1 - i0);
    }
    weights[i] = curv;
    sum_w += curv;
  }

  if (sum_w <= 1e-9) {
    return grid;
  }

  const std::size_t budget =
    std::min<std::size_t>(config_.adaptive_grid_max_points - grid.size(), base_n);
  const double mean_w = sum_w / static_cast<double>(weights.size());
  std::vector<double> refined;
  refined.reserve(grid.size() + budget);
  std::size_t added = 0;
  for (std::size_t i = 0; i + 1 < grid.size(); ++i) {
    refined.push_back(grid[i]);
    if (weights[i] > 1.15 * mean_w && added < budget) {
      const std::size_t sub = std::min<std::size_t>(
        4, 2 + static_cast<std::size_t>(weights[i] / std::max(1e-9, mean_w)));
      const std::size_t actual_sub = std::min<std::size_t>(sub, budget - added + 1);
      for (std::size_t k = 1; k < actual_sub; ++k) {
        const double r = static_cast<double>(k) / static_cast<double>(actual_sub);
        refined.push_back(grid[i] + r * (grid[i + 1] - grid[i]));
        ++added;
      }
    }
  }
  refined.push_back(1.0);
  return refined;
}

ToppraResult ToppraParameterizer::parameterize(
  const ParameterizedPath & path,
  double velocity_scale) const
{
  ToppraResult result;
  if (path.empty()) {
    result.failure = FailureCode::kInvalidInput;
    result.message = "ParameterizedPath is empty";
    return result;
  }
  if (!std::isfinite(velocity_scale) || velocity_scale <= 0.0 || velocity_scale > 1.0) {
    result.failure = FailureCode::kInvalidInput;
    result.message = "velocity_scale must be in (0, 1]";
    return result;
  }
  const bool use_torque = config_.enable_torque_constraints && (dynamics_ != nullptr);
  if (config_.enable_torque_constraints && dynamics_) {
    std::string reason;
    if (!dynamics_->config().hasValidDynamics(&reason)) {
      result.failure = FailureCode::kMissingDynamics;
      result.message = "dynamics model is incomplete: " + reason;
      return result;
    }
  }

  // スケール後の運動学上限（velocity_scale を縮小すると加速度も scale^2 で縮小し滑らかに減速させる）。
  State v_max(kDof);
  State a_max(kDof);
  for (std::size_t j = 0; j < kDof; ++j) {
    v_max[j] = config_.max_velocity[j] * velocity_scale;
    a_max[j] = config_.max_acceleration[j] * std::max(0.05, velocity_scale * velocity_scale);
  }

  // 静止経路（始点 == 終点）の特別処理
  if (path.isStationary()) {
    const State q0 = path.position(0.0);
    const State zero(kDof, 0.0);
    State tau0(kDof, 0.0);
    double min_torque_margin = std::numeric_limits<double>::infinity();
    if (dynamics_) {
      tau0 = dynamics_->inverseDynamics(q0, zero, zero);
      const TorqueBounds tb = dynamics_->torqueBounds(0, 0.0);
      if (!tb.feasible) {
        result.failure = FailureCode::kTimeParameterizationInfeasible;
        result.message = "theta1 torque bounds infeasible at stationary state";
        return result;
      }
      min_torque_margin = std::min(tb.max_torque - tau0[0], tau0[0] - tb.min_torque);
      if (use_torque && min_torque_margin < -kConstraintSlack) {
        result.failure = FailureCode::kTimeParameterizationInfeasible;
        result.message = "stationary state exceeds static gravity torque limit on theta1";
        result.minimum_torque_margin = min_torque_margin;
        result.bottleneck = "joint=0 type=static_torque time=0.0s";
        return result;
      }
      const double cont = dynamics_->theta1ContinuousAllowedTorque();
      if (use_torque && cont > 0.0 && std::abs(tau0[0]) > cont + kConstraintSlack) {
        result.failure = FailureCode::kThermalLimitViolation;
        result.message = "stationary state exceeds continuous RMS torque limit on theta1";
        result.minimum_torque_margin = min_torque_margin;
        return result;
      }
    }
    constexpr double kStationaryDuration = 0.05;
    result.profile.s = {0.0, 1.0};
    result.profile.s_dot = {0.0, 0.0};
    result.profile.s_ddot = {0.0, 0.0};
    result.profile.time = {0.0, kStationaryDuration};
    result.trajectory.joint_names = joint_names_;
    result.trajectory.points = {
      {q0, zero, zero, zero, tau0, 0.0},
      {q0, zero, zero, zero, tau0, kStationaryDuration}};
    result.duration = kStationaryDuration;
    result.theta1_peak_torque = std::abs(tau0[0]);
    result.theta1_rms_torque = std::abs(tau0[0]);
    result.minimum_torque_margin =
      std::isfinite(min_torque_margin) ? min_torque_margin : 0.0;
    result.minimum_velocity_margin = *std::min_element(v_max.begin(), v_max.end());
    result.minimum_acceleration_margin = *std::min_element(a_max.begin(), a_max.end());
    result.bottleneck = "stationary";
    return result;
  }

  const std::vector<double> s_grid = buildAdaptiveGrid(path);
  const std::size_t n_pts = s_grid.size();
  const std::size_t n_seg = n_pts - 1;

  std::vector<GridNodeData> nodes(n_pts);
  for (std::size_t k = 0; k < n_pts; ++k) {
    GridNodeData & nd = nodes[k];
    nd.s = s_grid[k];
    nd.q = path.position(nd.s);
    nd.dq = path.firstDerivative(nd.s);
    nd.d2q = path.secondDerivative(nd.s);
    if (!isFiniteState(nd.q) || !isFiniteState(nd.dq) || !isFiniteState(nd.d2q)) {
      result.failure = FailureCode::kNumericalError;
      result.message = "non-finite path derivatives on TOPP-RA grid";
      return result;
    }
    if (dynamics_) {
      nd.m = dynamics_->inertialTorque(nd.q, nd.dq);
      const State m_d2q = dynamics_->inertialTorque(nd.q, nd.d2q);
      const State cor = dynamics_->coriolisTorque(nd.q, nd.dq);
      nd.b.resize(kDof);
      for (std::size_t j = 0; j < kDof; ++j) {
        nd.b[j] = m_d2q[j] + cor[j];
      }
      nd.g = dynamics_->gravityTorque(nd.q);
    }
  }

  // 格子点 k、状態 x = s_dot^2 に加え、速度依存トルク曲線評価用の参照状態 x_ref（x と x_ref の厳しい方）
  // に対して許容 u = s_ddot 区間 [u_lo, u_hi] を計算する。
  const auto admissible_u = [&](
    std::size_t k, double x, double x_ref, double & u_lo, double & u_hi) -> bool {
      if (!std::isfinite(x) || x < -kConstraintSlack) {
        return false;
      }
      const double x_clamped = std::max(0.0, x);
      const double s_dot = std::sqrt(x_clamped);
      const double s_dot_ref = std::sqrt(std::max(x_clamped, std::max(0.0, x_ref)));
      const GridNodeData & nd = nodes[k];

      u_lo = -1e9;
      u_hi = 1e9;
      for (std::size_t j = 0; j < kDof; ++j) {
        if (std::abs(nd.dq[j]) * s_dot > v_max[j] + kConstraintSlack) {
          return false;
        }
        const double lo = -a_max[j] - nd.d2q[j] * x_clamped;
        const double hi = a_max[j] - nd.d2q[j] * x_clamped;
        if (!intersectLinear(nd.dq[j], lo, hi, u_lo, u_hi)) {
          return false;
        }
      }

      if (use_torque) {
        State qd(kDof);
        for (std::size_t j = 0; j < kDof; ++j) {
          qd[j] = nd.dq[j] * s_dot;
        }
        const State tau_f = dynamics_->frictionTorque(qd);

        for (std::size_t j = 0; j < kDof; ++j) {
          if (j != 0 && dynamics_->config().joint_peak_torque[j] <= 0.0) {
            continue;
          }
          const double qd_now = nd.dq[j] * s_dot;
          const double qd_hi = nd.dq[j] * s_dot_ref;
          const TorqueBounds tb_now = dynamics_->torqueBounds(j, qd_now);
          const TorqueBounds tb_hi = dynamics_->torqueBounds(j, qd_hi);
          if (!tb_now.feasible || !tb_hi.feasible) {
            return false;
          }
          // 保守側（狭い方）の上下限を採用する（仕様書 第11.4節）。
          const double tau_min = std::max(tb_now.min_torque, tb_hi.min_torque);
          const double tau_max = std::min(tb_now.max_torque, tb_hi.max_torque);
          const double c_term = nd.b[j] * x_clamped + nd.g[j] + tau_f[j];
          if (!intersectLinear(nd.m[j], tau_min - c_term, tau_max - c_term, u_lo, u_hi)) {
            return false;
          }
        }
      }
      if (u_lo > u_hi && u_lo - u_hi <= kConstraintSlack) {
        const double mid = 0.5 * (u_lo + u_hi);
        u_lo = mid;
        u_hi = mid;
      }
      return u_lo <= u_hi;
    };

  // 各格子 k での運動学・トルク曲線速度上限から決まる x_k の単独上界を求める。
  std::vector<double> x_cap(n_pts, 0.0);
  double curve_max_theta1_vel = std::numeric_limits<double>::infinity();
  if (use_torque) {
    const auto & t1 = dynamics_->config().theta1;
    if (!t1.positive_speed_torque_curve.empty() && !t1.negative_speed_torque_curve.empty()) {
      double v_curve = std::min(
        t1.positive_speed_torque_curve.back().speed,
        t1.negative_speed_torque_curve.back().speed);
      if (t1.curve_in_motor_side && t1.gear_ratio > 0.0) {
        v_curve /= t1.gear_ratio;
      }
      curve_max_theta1_vel = v_curve;
    }
  }

  for (std::size_t k = 0; k < n_pts; ++k) {
    double max_s_dot = 1e4;
    for (std::size_t j = 0; j < kDof; ++j) {
      const double dqj = std::abs(nodes[k].dq[j]);
      if (dqj > kEps) {
        max_s_dot = std::min(max_s_dot, v_max[j] / dqj);
      }
    }
    if (std::isfinite(curve_max_theta1_vel) && std::abs(nodes[k].dq[0]) > kEps) {
      max_s_dot = std::min(max_s_dot, curve_max_theta1_vel / std::abs(nodes[k].dq[0]));
    }
    const double x_upper = max_s_dot * max_s_dot;

    // x = 0 がまず実行可能か検査（静的重力トルク超過を即検出）
    double u_lo0 = 0.0;
    double u_hi0 = 0.0;
    if (!admissible_u(k, 0.0, 0.0, u_lo0, u_hi0)) {
      result.failure = FailureCode::kTimeParameterizationInfeasible;
      std::ostringstream oss;
      oss << "infeasible at s=" << nodes[k].s
          << " even at zero speed (static gravity torque or acceleration limit)";
      result.message = oss.str();
      result.bottleneck = "joint=0 type=static_torque s=" + std::to_string(nodes[k].s);
      return result;
    }

    // [0, x_upper] 内で admissible_u が成り立つ最大 x を二分探索する。
    double lo = 0.0;
    double hi = x_upper;
    if (admissible_u(k, hi, hi, u_lo0, u_hi0)) {
      lo = hi;
    } else {
      for (int iter = 0; iter < 40; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (admissible_u(k, mid, mid, u_lo0, u_hi0)) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
    }
    x_cap[k] = lo;
  }

  const double x_start = config_.start_velocity_zero ? 0.0 : (config_.start_s_dot * config_.start_s_dot);
  const double x_goal = config_.goal_velocity_zero ? 0.0 : (config_.goal_s_dot * config_.goal_s_dot);
  if (x_start > x_cap.front() + kConstraintSlack || x_goal > x_cap.back() + kConstraintSlack) {
    result.failure = FailureCode::kTimeParameterizationInfeasible;
    result.message = "boundary velocity exceeds state cap";
    return result;
  }

  // 区間 k で x_k から [next_lo, next_hi] 内のいずれかの x_{k+1} へ遷移可能か判定し、
  // 遷移可能な最大 x_{k+1} を out_best_next に返す。
  const auto max_reachable_next = [&](
    std::size_t k, double x_k, double x_ref_k, double x_ref_next,
    double next_lo, double next_hi, double & out_best_next) -> bool {
      const double two_ds = 2.0 * (s_grid[k + 1] - s_grid[k]);
      double u_lo_k = 0.0;
      double u_hi_k = 0.0;
      if (!admissible_u(k, x_k, x_ref_k, u_lo_k, u_hi_k)) {
        return false;
      }
      const double y_lo = std::max(next_lo, x_k + two_ds * u_lo_k);
      const double y_hi = std::min(next_hi, x_k + two_ds * u_hi_k);
      if (y_lo > y_hi + kConstraintSlack) {
        return false;
      }

      const auto check_y = [&](double y) -> bool {
          const double y_c = std::min(y_hi, std::max(y_lo, y));
          const double u = (y_c - x_k) / two_ds;
          double u_lo_next = 0.0;
          double u_hi_next = 0.0;
          if (!admissible_u(k + 1, y_c, std::max(y_c, x_ref_next), u_lo_next, u_hi_next)) {
            return false;
          }
          return u >= u_lo_next - 1e-7 && u <= u_hi_next + 1e-7;
        };

      if (check_y(y_hi)) {
        out_best_next = std::max(0.0, y_hi);
        return true;
      }
      // y_lo から y_hi の間で有効な領域を探し、その上限を二分探索する。
      double valid_y = -1.0;
      if (check_y(y_lo)) {
        valid_y = y_lo;
      } else {
        constexpr int kProbe = 16;
        for (int p = kProbe - 1; p >= 1; --p) {
          const double cand = y_lo + (static_cast<double>(p) / kProbe) * (y_hi - y_lo);
          if (check_y(cand)) {
            valid_y = cand;
            break;
          }
        }
      }
      if (valid_y < 0.0) {
        return false;
      }
      double lo = valid_y;
      double hi = y_hi;
      for (int iter = 0; iter < 30; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (check_y(mid)) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
      out_best_next = std::max(0.0, lo);
      return true;
    };

  // 速度―トルク曲線に対する保守的区分線形反復（仕様書 第11.4節）。
  std::vector<double> x_ref(n_pts, 0.0);
  std::vector<double> x_profile(n_pts, 0.0);
  bool converged = false;

  for (std::size_t curve_iter = 0; curve_iter < config_.max_curve_iterations; ++curve_iter) {
    std::vector<double> b_min(n_pts, 0.0);
    std::vector<double> b_max(n_pts, 0.0);
    b_min[n_seg] = x_goal;
    b_max[n_seg] = x_goal;

    // 1. 後向き可到達集合の計算
    for (std::size_t k = n_seg; k-- > 0;) {
      const double cap = x_cap[k];
      const double next_lo = b_min[k + 1];
      const double next_hi = b_max[k + 1];
      double dummy_next = 0.0;
      const auto can_reach = [&](double x_cand) {
          return max_reachable_next(
            k, x_cand, std::max(x_cand, x_ref[k]), x_ref[k + 1], next_lo, next_hi, dummy_next);
        };

      // 可到達なシード点を探索する
      double seed_lo = -1.0;
      double seed_hi = -1.0;
      if (can_reach(0.0)) {
        seed_lo = 0.0;
        seed_hi = 0.0;
      }
      if (can_reach(cap)) {
        if (seed_lo < 0.0) {
          seed_lo = cap;
        }
        seed_hi = cap;
      }
      if (seed_lo < 0.0 || seed_hi < 0.0) {
        constexpr int kSamples = 32;
        for (int s = 0; s <= kSamples; ++s) {
          const double cand = (static_cast<double>(s) / kSamples) * cap;
          if (can_reach(cand)) {
            if (seed_lo < 0.0) {
              seed_lo = cand;
            }
            seed_hi = cand;
          }
        }
      }
      if (seed_lo < 0.0) {
        result.failure = FailureCode::kTimeParameterizationInfeasible;
        result.message = "TOPP-RA backward pass empty reachable set at grid index " +
          std::to_string(k);
        result.bottleneck = "s=" + std::to_string(s_grid[k]) + " backward_infeasible";
        return result;
      }

      // 下限 b_min[k] の二分探索
      double lo = 0.0;
      double hi = seed_lo;
      for (int iter = 0; iter < 32 && hi - lo > 1e-9; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (can_reach(mid)) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      b_min[k] = hi;

      // 上限 b_max[k] の二分探索
      lo = seed_hi;
      hi = cap;
      for (int iter = 0; iter < 32 && hi - lo > 1e-9; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (can_reach(mid)) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
      b_max[k] = lo;
    }

    if (x_start < b_min[0] - 1e-6 || x_start > b_max[0] + 1e-6) {
      result.failure = FailureCode::kTimeParameterizationInfeasible;
      result.message = "start velocity is outside backward reachable set";
      result.bottleneck = "s=0.0 start_unreachable";
      return result;
    }

    // 2. 前向きパス
    std::vector<double> x_new(n_pts, 0.0);
    x_new[0] = x_start;
    for (std::size_t k = 0; k < n_seg; ++k) {
      double best_next = 0.0;
      if (!max_reachable_next(
          k, x_new[k], std::max(x_new[k], x_ref[k]), x_ref[k + 1],
          b_min[k + 1], b_max[k + 1], best_next))
      {
        result.failure = FailureCode::kTimeParameterizationInfeasible;
        result.message = "TOPP-RA forward pass failed at grid index " + std::to_string(k);
        return result;
      }
      x_new[k + 1] = best_next;
    }

    double max_diff = 0.0;
    for (std::size_t k = 0; k < n_pts; ++k) {
      if (!std::isfinite(x_new[k]) || x_new[k] < -kConstraintSlack) {
        result.failure = FailureCode::kNumericalError;
        result.message = "invalid x value during TOPP-RA iteration";
        return result;
      }
      x_new[k] = std::max(0.0, x_new[k]);
      max_diff = std::max(max_diff, std::abs(std::sqrt(x_new[k]) - std::sqrt(x_ref[k])));
    }
    x_profile = x_new;
    x_ref = x_new;

    if (!use_torque || (curve_iter >= 1 && max_diff <= config_.curve_convergence_tol)) {
      converged = true;
      break;
    }
  }

  if (!converged) {
    result.failure = FailureCode::kNumericalError;
    result.message = "TOPP-RA speed-torque curve iteration did not converge";
    return result;
  }

  // 3. 時刻列・経路速度・経路加速度と状態列の構築
  result.profile.s = s_grid;
  result.profile.s_dot.resize(n_pts, 0.0);
  result.profile.s_ddot.resize(n_pts, 0.0);
  result.profile.time.resize(n_pts, 0.0);

  for (std::size_t k = 0; k < n_pts; ++k) {
    result.profile.s_dot[k] = std::sqrt(std::max(0.0, x_profile[k]));
  }
  for (std::size_t k = 0; k < n_seg; ++k) {
    const double ds = s_grid[k + 1] - s_grid[k];
    result.profile.s_ddot[k] = (x_profile[k + 1] - x_profile[k]) / (2.0 * ds);
    const double denom = result.profile.s_dot[k] + result.profile.s_dot[k + 1];
    if (denom <= 1e-12) {
      // 両端とも速度 0 は非自明経路上では進行不能なので失敗とする（仕様書 第11.3, 11.4節）。
      result.failure = FailureCode::kTimeParameterizationInfeasible;
      result.message = "zero velocity interval encountered on non-stationary path";
      return result;
    }
    const double dt = 2.0 * ds / denom;
    if (!std::isfinite(dt) || dt <= 0.0) {
      result.failure = FailureCode::kNumericalError;
      result.message = "non-positive or non-finite segment duration";
      return result;
    }
    result.profile.time[k + 1] = result.profile.time[k] + dt;
  }
  result.profile.s_ddot[n_seg] = config_.goal_velocity_zero ? 0.0 : result.profile.s_ddot[n_seg - 1];

  // 4. 各格子点の運動学量・逆動力学トルク・余裕・ボトルネックの評価
  result.trajectory.joint_names = joint_names_;
  result.trajectory.points.resize(n_pts);
  result.minimum_torque_margin = std::numeric_limits<double>::infinity();
  result.minimum_velocity_margin = std::numeric_limits<double>::infinity();
  result.minimum_acceleration_margin = std::numeric_limits<double>::infinity();
  double min_normalized_margin = std::numeric_limits<double>::infinity();

  for (std::size_t k = 0; k < n_pts; ++k) {
    DynamicState & pt = result.trajectory.points[k];
    pt.time = result.profile.time[k];
    pt.position = nodes[k].q;
    pt.velocity.resize(kDof);
    pt.acceleration.resize(kDof);
    pt.jerk.assign(kDof, 0.0);

    const double sd = result.profile.s_dot[k];
    const double sdd = result.profile.s_ddot[k];
    for (std::size_t j = 0; j < kDof; ++j) {
      pt.velocity[j] = nodes[k].dq[j] * sd;
      pt.acceleration[j] = nodes[k].dq[j] * sdd + nodes[k].d2q[j] * (sd * sd);

      const double v_margin = config_.max_velocity[j] - std::abs(pt.velocity[j]);
      const double a_margin = config_.max_acceleration[j] - std::abs(pt.acceleration[j]);
      result.minimum_velocity_margin = std::min(result.minimum_velocity_margin, v_margin);
      result.minimum_acceleration_margin = std::min(result.minimum_acceleration_margin, a_margin);

      const double v_norm = v_margin / config_.max_velocity[j];
      if (v_norm < min_normalized_margin) {
        min_normalized_margin = v_norm;
        result.bottleneck = "joint=" + std::to_string(j) + " type=velocity time=" +
          std::to_string(pt.time) + "s";
      }
      const double a_norm = a_margin / config_.max_acceleration[j];
      if (a_norm < min_normalized_margin) {
        min_normalized_margin = a_norm;
        result.bottleneck = "joint=" + std::to_string(j) + " type=acceleration time=" +
          std::to_string(pt.time) + "s";
      }
    }

    if (dynamics_) {
      pt.torque = dynamics_->inverseDynamics(pt.position, pt.velocity, pt.acceleration);
      result.theta1_peak_torque = std::max(result.theta1_peak_torque, std::abs(pt.torque[0]));
      const TorqueBounds tb = dynamics_->torqueBounds(0, pt.velocity[0]);
      if (!tb.feasible) {
        result.failure = FailureCode::kTorqueLimitViolation;
        result.message = "theta1 velocity outside torque curve range";
        return result;
      }
      const double t_margin = std::min(tb.max_torque - pt.torque[0], pt.torque[0] - tb.min_torque);
      result.minimum_torque_margin = std::min(result.minimum_torque_margin, t_margin);
      const double span = std::max(1e-6, 0.5 * (tb.max_torque - tb.min_torque));
      const double t_norm = t_margin / span;
      if (t_norm < min_normalized_margin) {
        min_normalized_margin = t_norm;
        result.bottleneck = "joint=0 type=torque time=" + std::to_string(pt.time) + "s";
      }
    } else {
      pt.torque.assign(kDof, 0.0);
    }
  }

  // 離散時刻列の加速度差分から初期 jerk を格納する。
  for (std::size_t k = 0; k < n_seg; ++k) {
    const double dt = result.trajectory.points[k + 1].time - result.trajectory.points[k].time;
    for (std::size_t j = 0; j < kDof; ++j) {
      result.trajectory.points[k].jerk[j] =
        (result.trajectory.points[k + 1].acceleration[j] -
        result.trajectory.points[k].acceleration[j]) / dt;
    }
  }
  result.trajectory.points[n_seg].jerk = result.trajectory.points[n_seg - 1].jerk;

  result.duration = result.trajectory.points.back().time;
  if (dynamics_ && result.duration > 0.0) {
    double integral_sq = 0.0;
    for (std::size_t k = 0; k < n_seg; ++k) {
      const double dt = result.trajectory.points[k + 1].time - result.trajectory.points[k].time;
      const double t0 = result.trajectory.points[k].torque[0];
      const double t1 = result.trajectory.points[k + 1].torque[0];
      integral_sq += 0.5 * (t0 * t0 + t1 * t1) * dt;
    }
    result.theta1_rms_torque = std::sqrt(integral_sq / result.duration);
  }
  if (!std::isfinite(result.minimum_torque_margin)) {
    result.minimum_torque_margin = 0.0;
  }
  return result;
}

}  // namespace path_planner
