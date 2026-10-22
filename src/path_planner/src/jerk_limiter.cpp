#include "path_planner/jerk_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace path_planner
{
namespace
{
constexpr double kSlack = 1e-6;

// 時刻 [0, T] を [0, 1] に正規化した無次元時刻 tau in [0, 1] に対する単調増加 C2 ワープ関数 w(tau):
//   w(0) = 0, w'(0) = 0, w''(0) = 0
//   w(1) = 1, w'(1) = 0, w''(1) = 0
// 両端 blend 幅 eps in (0, 0.45] のみ 5次多項式で立ち上げ・立ち下げ、中央 [eps, 1-eps] は等速直線とする。
// これにより w(tau) は全域で C2 連続かつ厳密に単調増加 (w'(tau) >= 0) となり、高周波振動を一切生まない。
class BoundarySmoothWarp
{
public:
  explicit BoundarySmoothWarp(double eps = 0.15)
  : eps_(std::min(0.45, std::max(0.02, eps))),
    v_cruise_(1.0 / (1.0 - eps_)) {}

  void evaluate(double tau, double & w, double & dw, double & d2w) const
  {
    const double t = std::min(1.0, std::max(0.0, tau));
    if (t <= eps_) {
      const double u = t / eps_;
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double u4 = u3 * u;
      // f(0)=0, f'(0)=0, f''(0)=0, f(1)=0.5, f'(1)=1, f''(1)=0 -> f(u) = u^3 - 0.5 u^4
      w = eps_ * v_cruise_ * (u3 - 0.5 * u4);
      dw = v_cruise_ * (3.0 * u2 - 2.0 * u3);
      d2w = (v_cruise_ / eps_) * (6.0 * u - 6.0 * u2);
    } else if (t < 1.0 - eps_) {
      w = 0.5 * eps_ * v_cruise_ + v_cruise_ * (t - eps_);
      dw = v_cruise_;
      d2w = 0.0;
    } else {
      const double u = (1.0 - t) / eps_;
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double u4 = u3 * u;
      w = 1.0 - eps_ * v_cruise_ * (u3 - 0.5 * u4);
      dw = v_cruise_ * (3.0 * u2 - 2.0 * u3);
      d2w = -(v_cruise_ / eps_) * (6.0 * u - 6.0 * u2);
    }
  }

private:
  double eps_;
  double v_cruise_;
};

// ワープ後時刻 w in [0, 1] に対する C2 自然3次スプライン s(w)。
struct CubicTimeSpline
{
  std::vector<double> w_knots;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;

  void evaluate(double w, double & s, double & ds_dw, double & d2s_dw2) const
  {
    const double wc = std::min(1.0, std::max(0.0, w));
    const std::size_t n = a.size();
    std::size_t idx = 0;
    if (wc >= w_knots.back()) {
      idx = n - 1;
    } else {
      auto it = std::upper_bound(w_knots.begin(), w_knots.end(), wc);
      if (it != w_knots.begin()) {
        idx = static_cast<std::size_t>(std::distance(w_knots.begin(), it) - 1);
      }
      if (idx >= n) {
        idx = n - 1;
      }
    }
    const double u = wc - w_knots[idx];
    s = a[idx] + u * (b[idx] + u * (c[idx] + u * d[idx]));
    ds_dw = b[idx] + u * (2.0 * c[idx] + 3.0 * d[idx] * u);
    d2s_dw2 = 2.0 * c[idx] + 6.0 * d[idx] * u;
  }
};

// 節点 (w_k, s_k) から単調増加 C2 3次スプラインを構築する。
CubicTimeSpline buildCubicTimeSpline(
  const std::vector<double> & s_grid,
  const std::vector<double> & dt_list)
{
  const std::size_t n = dt_list.size();
  std::vector<double> w_knots(n + 1, 0.0);
  double total_t = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    total_t += dt_list[k];
    w_knots[k + 1] = total_t;
  }
  for (double & w : w_knots) {
    w /= total_t;
  }
  w_knots.front() = 0.0;
  w_knots.back() = 1.0;

  std::vector<double> h(n);
  for (std::size_t k = 0; k < n; ++k) {
    h[k] = w_knots[k + 1] - w_knots[k];
  }

  CubicTimeSpline sp;
  sp.w_knots = w_knots;
  sp.a.resize(n);
  sp.b.resize(n);
  sp.c.resize(n);
  sp.d.resize(n);

  if (n == 1) {
    sp.a[0] = s_grid[0];
    sp.b[0] = (s_grid[1] - s_grid[0]) / h[0];
    return sp;
  }

  // 自然3次スプラインを解く
  std::vector<double> alpha(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    alpha[i] = (3.0 / h[i]) * (s_grid[i + 1] - s_grid[i]) -
      (3.0 / h[i - 1]) * (s_grid[i] - s_grid[i - 1]);
  }
  std::vector<double> l(n + 1, 1.0);
  std::vector<double> mu(n + 1, 0.0);
  std::vector<double> z(n + 1, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    l[i] = 2.0 * (w_knots[i + 1] - w_knots[i - 1]) - h[i - 1] * mu[i - 1];
    mu[i] = h[i] / l[i];
    z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
  }
  std::vector<double> c_vec(n + 1, 0.0);
  for (std::size_t idx = n; idx-- > 0;) {
    c_vec[idx] = z[idx] - mu[idx] * c_vec[idx + 1];
    sp.a[idx] = s_grid[idx];
    sp.b[idx] = (s_grid[idx + 1] - s_grid[idx]) / h[idx] -
      h[idx] * (c_vec[idx + 1] + 2.0 * c_vec[idx]) / 3.0;
    sp.c[idx] = c_vec[idx];
    sp.d[idx] = (c_vec[idx + 1] - c_vec[idx]) / (3.0 * h[idx]);
  }
  return sp;
}
}  // namespace

JerkLimiter::JerkLimiter(
  JerkLimiterConfig config,
  std::shared_ptr<const RneaDynamicsModel> dynamics,
  std::vector<std::string> joint_names)
: config_(std::move(config)),
  dynamics_(std::move(dynamics)),
  joint_names_(std::move(joint_names))
{
  if (config_.max_velocity.size() != kDof ||
    config_.max_acceleration.size() != kDof ||
    config_.max_jerk.size() != kDof)
  {
    throw std::invalid_argument("JerkLimiter velocity/acceleration/jerk limits must have 4 elements");
  }
  for (std::size_t j = 0; j < kDof; ++j) {
    if (!std::isfinite(config_.max_velocity[j]) || config_.max_velocity[j] <= 0.0 ||
      !std::isfinite(config_.max_acceleration[j]) || config_.max_acceleration[j] <= 0.0 ||
      !std::isfinite(config_.max_jerk[j]) || config_.max_jerk[j] <= 0.0)
    {
      throw std::invalid_argument("JerkLimiter kinematic limits must be positive");
    }
  }
  if (!std::isfinite(config_.sample_dt) || config_.sample_dt <= 0.0) {
    throw std::invalid_argument("JerkLimiter sample_dt must be > 0");
  }
  if (config_.max_iterations == 0) {
    throw std::invalid_argument("JerkLimiter max_iterations must be >= 1");
  }
}

JerkLimitResult JerkLimiter::limit(
  const ParameterizedPath & path,
  const ToppraGridProfile & topp_profile) const
{
  JerkLimitResult result;
  if (path.empty() || topp_profile.s.size() < 2 ||
    topp_profile.time.size() != topp_profile.s.size())
  {
    result.failure = FailureCode::kInvalidInput;
    result.message = "invalid path or TOPP-RA profile passed to JerkLimiter";
    return result;
  }

  // 静止経路の扱い
  if (path.isStationary()) {
    const State q0 = path.position(0.0);
    const State zero(kDof, 0.0);
    State tau0(kDof, 0.0);
    double min_margin = 0.0;
    if (dynamics_) {
      tau0 = dynamics_->inverseDynamics(q0, zero, zero);
      const TorqueBounds tb = dynamics_->torqueBounds(0, 0.0);
      if (!tb.feasible) {
        result.failure = FailureCode::kJerkLimitInfeasible;
        result.message = "theta1 torque bounds infeasible at stationary state";
        return result;
      }
      min_margin = std::min(tb.max_torque - tau0[0], tau0[0] - tb.min_torque);
      if (config_.enforce_torque_limits && min_margin < -kSlack) {
        result.failure = FailureCode::kJerkLimitInfeasible;
        result.message = "stationary state violates theta1 torque limit";
        result.minimum_torque_margin = min_margin;
        return result;
      }
    }
    const double dt = std::max(0.01, config_.sample_dt * 10.0);
    result.trajectory.joint_names = joint_names_;
    result.trajectory.points = {
      {q0, zero, zero, zero, tau0, 0.0},
      {q0, zero, zero, zero, tau0, dt}};
    result.duration = dt;
    result.theta1_peak_torque = std::abs(tau0[0]);
    result.theta1_rms_torque = std::abs(tau0[0]);
    result.minimum_torque_margin = min_margin;
    return result;
  }

  const std::size_t n_pts = topp_profile.s.size();
  const std::size_t n_seg = n_pts - 1;
  std::vector<double> dt_list(n_seg, 0.0);
  for (std::size_t k = 0; k < n_seg; ++k) {
    const double dt = topp_profile.time[k + 1] - topp_profile.time[k];
    if (!std::isfinite(dt) || dt <= 0.0) {
      result.failure = FailureCode::kNumericalError;
      result.message = "non-positive dt in TOPP-RA profile";
      return result;
    }
    dt_list[k] = dt;
  }

  // 各区間の逆速度 (dt_k / ds_k) の急峻な段差・V字谷を保守側（遅い側）包絡 + 平滑化で滑らかにし、
  // s(w) の高次微分 (d2s/dw2, d3s/dw3) スパイクを抑える。
  std::vector<double> ds_list(n_seg, 0.0);
  std::vector<double> slowness(n_seg, 0.0);
  for (std::size_t k = 0; k < n_seg; ++k) {
    ds_list[k] = std::max(1e-9, topp_profile.s[k + 1] - topp_profile.s[k]);
    slowness[k] = dt_list[k] / ds_list[k];
  }
  const int smooth_passes = std::max<int>(6, static_cast<int>(n_seg / 10));
  for (int pass = 0; pass < smooth_passes; ++pass) {
    std::vector<double> next_s = slowness;
    for (std::size_t k = 1; k + 1 < n_seg; ++k) {
      const double avg = 0.25 * slowness[k - 1] + 0.5 * slowness[k] + 0.25 * slowness[k + 1];
      // TOPP-RA の速度上限を破らないよう、最初の半分のパスは保守側（遅い側）を優先する
      next_s[k] = (pass < smooth_passes / 2) ? std::max(slowness[k], avg) : avg;
    }
    slowness = std::move(next_s);
  }
  for (std::size_t k = 0; k < n_seg; ++k) {
    dt_list[k] = slowness[k] * ds_list[k];
  }

  const BoundarySmoothWarp warp(0.15);
  // BoundarySmoothWarp の巡航倍率 v_cruise = 1 / (1 - 0.15) ~= 1.176 を初期時間で補償する。
  for (double & dt : dt_list) {
    dt *= 1.18;
  }

  const auto evaluate_profile = [&](
    const CubicTimeSpline & sp,
    double total_t,
    double eval_dt,
    TimedTrajectory & out_traj,
    std::vector<double> & seg_scale_needed,
    double & out_peak_tau1,
    double & out_rms_tau1,
    double & out_min_margin) -> bool {
      seg_scale_needed.assign(n_seg, 1.0);
      out_traj.joint_names = joint_names_;
      out_traj.points.clear();

      const std::size_t steps = std::max<std::size_t>(
        20, static_cast<std::size_t>(std::ceil(total_t / eval_dt)));
      out_traj.points.reserve(steps + 1);
      std::vector<std::size_t> point_seg_idx(steps + 1, 0);

      for (std::size_t i = 0; i <= steps; ++i) {
        const double tau = static_cast<double>(i) / static_cast<double>(steps);
        const double t = tau * total_t;
        double w = 0.0;
        double dw_dtau = 0.0;
        double d2w_dtau2 = 0.0;
        warp.evaluate(tau, w, dw_dtau, d2w_dtau2);

        double s = 0.0;
        double ds_dw = 0.0;
        double d2s_dw2 = 0.0;
        sp.evaluate(w, s, ds_dw, d2s_dw2);
        s = std::min(1.0, std::max(0.0, s));
        ds_dw = std::max(0.0, ds_dw);

        const double inv_t = 1.0 / total_t;
        const double sd = ds_dw * dw_dtau * inv_t;
        const double sdd =
          (d2s_dw2 * (dw_dtau * dw_dtau) + ds_dw * d2w_dtau2) * (inv_t * inv_t);

        DynamicState pt;
        pt.time = t;
        pt.position = path.position(s);
        const State dq = path.firstDerivative(s);
        const State d2q = path.secondDerivative(s);
        pt.velocity.resize(kDof);
        pt.acceleration.resize(kDof);
        pt.jerk.assign(kDof, 0.0);
        for (std::size_t j = 0; j < kDof; ++j) {
          pt.velocity[j] = dq[j] * sd;
          pt.acceleration[j] = dq[j] * sdd + d2q[j] * (sd * sd);
        }
        out_traj.points.push_back(std::move(pt));

        auto it = std::upper_bound(sp.w_knots.begin(), sp.w_knots.end(), w);
        std::size_t seg_idx = 0;
        if (it != sp.w_knots.begin()) {
          seg_idx = static_cast<std::size_t>(std::distance(sp.w_knots.begin(), it) - 1);
        }
        point_seg_idx[i] = std::min(n_seg - 1, seg_idx);
      }

      // 始端・終端の境界条件を厳密に固定する
      out_traj.points.front().position = path.position(0.0);
      out_traj.points.back().position = path.position(1.0);
      if (config_.start_velocity_zero) {
        out_traj.points.front().velocity.assign(kDof, 0.0);
      }
      if (config_.goal_velocity_zero) {
        out_traj.points.back().velocity.assign(kDof, 0.0);
      }
      if (config_.start_acceleration_zero) {
        out_traj.points.front().acceleration.assign(kDof, 0.0);
      }
      if (config_.goal_acceleration_zero) {
        out_traj.points.back().acceleration.assign(kDof, 0.0);
      }

      const std::size_t m_pts = out_traj.points.size();
      for (std::size_t i = 0; i + 1 < m_pts; ++i) {
        const double dt = out_traj.points[i + 1].time - out_traj.points[i].time;
        for (std::size_t j = 0; j < kDof; ++j) {
          out_traj.points[i].jerk[j] =
            (out_traj.points[i + 1].acceleration[j] - out_traj.points[i].acceleration[j]) / dt;
        }
      }
      out_traj.points.back().jerk = out_traj.points[m_pts - 2].jerk;

      bool all_ok = true;
      out_peak_tau1 = 0.0;
      out_min_margin = std::numeric_limits<double>::infinity();

      for (std::size_t i = 0; i < m_pts; ++i) {
        DynamicState & pt = out_traj.points[i];
        const std::size_t seg_idx = point_seg_idx[i];

        double scale = 1.0;
        for (std::size_t j = 0; j < kDof; ++j) {
          const double rv = std::abs(pt.velocity[j]) / config_.max_velocity[j];
          const double ra = std::abs(pt.acceleration[j]) / config_.max_acceleration[j];
          const double rj = std::abs(pt.jerk[j]) / config_.max_jerk[j];
          if (rv > 1.0 + kSlack) {
            all_ok = false;
            scale = std::max(scale, rv);
          }
          if (ra > 1.0 + kSlack) {
            all_ok = false;
            scale = std::max(scale, std::sqrt(ra));
          }
          if (rj > 1.0 + kSlack) {
            all_ok = false;
            scale = std::max(scale, std::cbrt(rj));
          }
        }

        if (dynamics_) {
          pt.torque = dynamics_->inverseDynamics(pt.position, pt.velocity, pt.acceleration);
          out_peak_tau1 = std::max(out_peak_tau1, std::abs(pt.torque[0]));
          const TorqueBounds tb = dynamics_->torqueBounds(0, pt.velocity[0]);
          if (!tb.feasible) {
            if (config_.enforce_torque_limits) {
              all_ok = false;
              scale = std::max(scale, 1.2);
            }
          } else {
            const double margin = std::min(tb.max_torque - pt.torque[0], pt.torque[0] - tb.min_torque);
            out_min_margin = std::min(out_min_margin, margin);
            if (config_.enforce_torque_limits && margin < -kSlack) {
              all_ok = false;
              const double g0 = dynamics_->gravityTorque(pt.position)[0];
              const double dyn_part = std::abs(pt.torque[0] - g0);
              const double allowed_dyn = std::max(
                0.0, (pt.torque[0] >= 0.0 ? (tb.max_torque - g0) : (g0 - tb.min_torque)));
              if (allowed_dyn <= 1e-6 || dyn_part <= 1e-6) {
                scale = std::max(scale, 1.4);
              } else {
                scale = std::max(scale, std::sqrt(dyn_part / allowed_dyn) * 1.05);
              }
            }
          }

          for (std::size_t j = 1; j < kDof; ++j) {
            if (dynamics_->config().joint_peak_torque[j] <= 0.0) {
              continue;
            }
            const TorqueBounds tbj = dynamics_->torqueBounds(j, pt.velocity[j]);
            const double mj = std::min(tbj.max_torque - pt.torque[j], pt.torque[j] - tbj.min_torque);
            out_min_margin = std::min(out_min_margin, mj);
            if (config_.enforce_torque_limits && (!tbj.feasible || mj < -kSlack)) {
              all_ok = false;
              scale = std::max(scale, 1.15);
            }
          }
        } else {
          pt.torque.assign(kDof, 0.0);
        }

        seg_scale_needed[seg_idx] = std::max(seg_scale_needed[seg_idx], scale);
      }

      out_rms_tau1 = 0.0;
      if (dynamics_ && total_t > 0.0) {
        double sq_int = 0.0;
        for (std::size_t i = 0; i + 1 < m_pts; ++i) {
          const double dt = out_traj.points[i + 1].time - out_traj.points[i].time;
          const double t0 = out_traj.points[i].torque[0];
          const double t1 = out_traj.points[i + 1].torque[0];
          sq_int += 0.5 * (t0 * t0 + t1 * t1) * dt;
        }
        out_rms_tau1 = std::sqrt(sq_int / total_t);
        const double cont = dynamics_->theta1ContinuousAllowedTorque();
        if (config_.enforce_torque_limits && cont > 0.0 && out_rms_tau1 > cont + kSlack) {
          all_ok = false;
          const double rms_ratio = out_rms_tau1 / cont;
          for (double & s : seg_scale_needed) {
            s = std::max(s, rms_ratio * 1.05);
          }
        }
      }
      if (!std::isfinite(out_min_margin)) {
        out_min_margin = 0.0;
      }
      return all_ok;
    };

  TimedTrajectory candidate_traj;
  std::vector<double> seg_scales;
  double peak_tau1 = 0.0;
  double rms_tau1 = 0.0;
  double min_margin = 0.0;

  for (std::size_t iter = 0; iter < config_.max_iterations; ++iter) {
    result.iterations_used = iter + 1;
    double total_t = 0.0;
    for (double dt : dt_list) {
      total_t += dt;
    }
    const CubicTimeSpline sp = buildCubicTimeSpline(topp_profile.s, dt_list);

    const double coarse_dt = std::max(config_.sample_dt, total_t / 250.0);
    if (evaluate_profile(sp, total_t, coarse_dt, candidate_traj, seg_scales, peak_tau1, rms_tau1, min_margin)) {
      if (evaluate_profile(sp, total_t, config_.sample_dt, candidate_traj, seg_scales, peak_tau1, rms_tau1, min_margin)) {
        result.trajectory = std::move(candidate_traj);
        result.duration = result.trajectory.points.back().time;
        result.theta1_peak_torque = peak_tau1;
        result.theta1_rms_torque = rms_tau1;
        result.minimum_torque_margin = min_margin;
        return result;
      }
    }

    // 違反区間とその前後の時間幅を滑らかに拡大し、局所段差による二次的な躍度増大を防ぐ（仕様書 第12.1節）。
    double max_req_scale = 1.0;
    for (double s : seg_scales) {
      max_req_scale = std::max(max_req_scale, s);
    }
    const double global_base = 1.0 + 0.35 * (std::min(1.8, max_req_scale) - 1.0);
    std::vector<double> smoothed_scale(n_seg, global_base);
    const std::size_t radius = std::max<std::size_t>(4, n_seg / 12);
    for (std::size_t k = 0; k < n_seg; ++k) {
      if (seg_scales[k] > 1.0) {
        const double factor = std::min(1.7, seg_scales[k] * 1.05 + 0.02);
        const std::size_t lo = k >= radius ? (k - radius) : 0;
        const std::size_t hi = std::min(n_seg - 1, k + radius);
        for (std::size_t m = lo; m <= hi; ++m) {
          const double dist = static_cast<double>(m > k ? (m - k) : (k - m)) /
            static_cast<double>(radius + 1);
          const double weight = 0.5 * (1.0 + std::cos(kPi * dist));
          const double local_f = global_base + std::max(0.0, factor - global_base) * weight;
          smoothed_scale[m] = std::max(smoothed_scale[m], local_f);
        }
      }
    }
    for (std::size_t k = 0; k < n_seg; ++k) {
      slowness[k] = (dt_list[k] * smoothed_scale[k]) / ds_list[k];
    }
    for (int pass = 0; pass < 6; ++pass) {
      std::vector<double> next_s = slowness;
      for (std::size_t k = 1; k + 1 < n_seg; ++k) {
        next_s[k] = 0.25 * slowness[k - 1] + 0.5 * slowness[k] + 0.25 * slowness[k + 1];
      }
      slowness = std::move(next_s);
    }
    for (std::size_t k = 0; k < n_seg; ++k) {
      dt_list[k] = slowness[k] * ds_list[k];
    }
  }

  result.failure = FailureCode::kJerkLimitInfeasible;
  result.message = "JerkLimiter could not satisfy jerk/kinematic/torque limits within max_iterations";
  result.theta1_peak_torque = peak_tau1;
  result.theta1_rms_torque = rms_tau1;
  result.minimum_torque_margin = min_margin;
  return result;
}

}  // namespace path_planner
