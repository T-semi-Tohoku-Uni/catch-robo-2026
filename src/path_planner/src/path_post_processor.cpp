#include "path_planner/path_post_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace path_planner
{
namespace
{
double maxAbsDiff(const State & a, const State & b)
{
  double m = 0.0;
  for (std::size_t i = 0; i < kDof; ++i) {
    m = std::max(m, std::abs(a[i] - b[i]));
  }
  return m;
}

double euclideanDistance(const State & a, const State & b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < kDof; ++i) {
    const double d = a[i] - b[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// 連続する重複点を除去する（ただし最低2点は残す）。
Path deduplicateWaypoints(const Path & raw)
{
  if (raw.size() <= 2) {
    return raw;
  }
  Path cleaned;
  cleaned.reserve(raw.size());
  cleaned.push_back(raw.front());
  for (std::size_t i = 1; i + 1 < raw.size(); ++i) {
    if (euclideanDistance(raw[i], cleaned.back()) > 1e-9) {
      cleaned.push_back(raw[i]);
    }
  }
  if (euclideanDistance(raw.back(), cleaned.back()) > 1e-9 || cleaned.size() == 1) {
    cleaned.push_back(raw.back());
  } else {
    cleaned.back() = raw.back();
  }
  return cleaned;
}

// 折れ線の各辺を subdivisions 分割して中間節点を挿入する（スプラインのオーバーシュート抑制用）。
Path subdividePolyline(const Path & polyline, std::size_t subdivisions)
{
  if (subdivisions <= 1 || polyline.size() < 2) {
    return polyline;
  }
  Path out;
  out.reserve((polyline.size() - 1) * subdivisions + 1);
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    for (std::size_t k = 0; k < subdivisions; ++k) {
      const double r = static_cast<double>(k) / static_cast<double>(subdivisions);
      State q(kDof);
      for (std::size_t j = 0; j < kDof; ++j) {
        q[j] = polyline[i][j] + r * (polyline[i + 1][j] - polyline[i][j]);
      }
      out.push_back(std::move(q));
    }
  }
  out.push_back(polyline.back());
  return out;
}

// 折れ線上の弦長パラメータ s in [0, 1] での線形補間位置を返す。
State evaluatePolyline(
  const Path & polyline,
  const std::vector<double> & poly_knots,
  double s)
{
  if (s <= 0.0) {
    return polyline.front();
  }
  if (s >= 1.0) {
    return polyline.back();
  }
  auto it = std::upper_bound(poly_knots.begin(), poly_knots.end(), s);
  std::size_t idx = 0;
  if (it != poly_knots.begin()) {
    idx = static_cast<std::size_t>(std::distance(poly_knots.begin(), it) - 1);
  }
  if (idx + 1 >= polyline.size()) {
    idx = polyline.size() - 2;
  }
  const double h = poly_knots[idx + 1] - poly_knots[idx];
  const double ratio = h > 1e-12 ? (s - poly_knots[idx]) / h : 0.0;
  State q(kDof);
  for (std::size_t j = 0; j < kDof; ++j) {
    q[j] = polyline[idx][j] + ratio * (polyline[idx + 1][j] - polyline[idx][j]);
  }
  return q;
}

std::vector<double> computeChordKnots(const Path & pts)
{
  std::vector<double> knots(pts.size(), 0.0);
  double total = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    total += euclideanDistance(pts[i - 1], pts[i]);
    knots[i] = total;
  }
  if (total <= 1e-12) {
    for (std::size_t i = 0; i < pts.size(); ++i) {
      knots[i] = static_cast<double>(i) / static_cast<double>(pts.size() - 1);
    }
    return knots;
  }
  for (double & s : knots) {
    s /= total;
  }
  knots.front() = 0.0;
  knots.back() = 1.0;
  return knots;
}
}  // namespace

ParameterizedPath::ParameterizedPath(
  std::vector<double> knots,
  std::vector<SegmentCoeffs> segments)
: knots_(std::move(knots)), segments_(std::move(segments))
{
  if (segments_.empty() || knots_.size() != segments_.size() + 1) {
    throw std::invalid_argument("ParameterizedPath knots/segments size mismatch");
  }
}

ParameterizedPath ParameterizedPath::makeStationary(const State & q)
{
  if (!isFiniteState(q)) {
    throw std::invalid_argument("makeStationary requires a valid 4D state");
  }
  SegmentCoeffs seg{};
  for (std::size_t j = 0; j < kDof; ++j) {
    seg.a[j] = q[j];
  }
  return ParameterizedPath({0.0, 1.0}, {seg});
}

ParameterizedPath ParameterizedPath::fromWaypoints(const Path & waypoints)
{
  if (waypoints.size() < 2) {
    throw std::invalid_argument("fromWaypoints requires at least 2 waypoints");
  }
  for (const auto & q : waypoints) {
    if (!isFiniteState(q)) {
      throw std::invalid_argument("fromWaypoints contains invalid state");
    }
  }
  const Path pts = deduplicateWaypoints(waypoints);
  const std::size_t n = pts.size() - 1;
  if (n == 1 && euclideanDistance(pts.front(), pts.back()) <= 1e-12) {
    return makeStationary(pts.front());
  }

  const std::vector<double> knots = computeChordKnots(pts);
  std::vector<double> h(n);
  for (std::size_t i = 0; i < n; ++i) {
    h[i] = knots[i + 1] - knots[i];
    if (h[i] <= 1e-12) {
      throw std::invalid_argument("non-increasing spline knots");
    }
  }

  std::vector<SegmentCoeffs> segments(n);
  if (n == 1) {
    for (std::size_t j = 0; j < kDof; ++j) {
      segments[0].a[j] = pts[0][j];
      segments[0].b[j] = (pts[1][j] - pts[0][j]) / h[0];
      segments[0].c[j] = 0.0;
      segments[0].d[j] = 0.0;
    }
    return ParameterizedPath(knots, std::move(segments));
  }

  // 自然3次スプライン（両端の2階微分 = 0）を Thomas 法で解く。
  for (std::size_t j = 0; j < kDof; ++j) {
    std::vector<double> alpha(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
      alpha[i] = (3.0 / h[i]) * (pts[i + 1][j] - pts[i][j]) -
        (3.0 / h[i - 1]) * (pts[i][j] - pts[i - 1][j]);
    }
    std::vector<double> l(n + 1, 1.0);
    std::vector<double> mu(n + 1, 0.0);
    std::vector<double> z(n + 1, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
      l[i] = 2.0 * (knots[i + 1] - knots[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    std::vector<double> c(n + 1, 0.0);
    for (std::size_t idx = n; idx-- > 0;) {
      c[idx] = z[idx] - mu[idx] * c[idx + 1];
      const double b = (pts[idx + 1][j] - pts[idx][j]) / h[idx] -
        h[idx] * (c[idx + 1] + 2.0 * c[idx]) / 3.0;
      const double d = (c[idx + 1] - c[idx]) / (3.0 * h[idx]);
      segments[idx].a[j] = pts[idx][j];
      segments[idx].b[j] = b;
      segments[idx].c[j] = c[idx];
      segments[idx].d[j] = d;
    }
  }
  return ParameterizedPath(knots, std::move(segments));
}

bool ParameterizedPath::isStationary(double tol) const
{
  for (const auto & seg : segments_) {
    for (std::size_t j = 0; j < kDof; ++j) {
      if (std::abs(seg.b[j]) > tol || std::abs(seg.c[j]) > tol || std::abs(seg.d[j]) > tol) {
        return false;
      }
    }
  }
  return true;
}

std::size_t ParameterizedPath::findSegment(double s, double & u) const
{
  if (segments_.empty()) {
    throw std::logic_error("ParameterizedPath is empty");
  }
  const double clamped = std::min(1.0, std::max(0.0, s));
  if (clamped >= knots_.back()) {
    const std::size_t last = segments_.size() - 1;
    u = knots_[last + 1] - knots_[last];
    return last;
  }
  auto it = std::upper_bound(knots_.begin(), knots_.end(), clamped);
  std::size_t idx = 0;
  if (it != knots_.begin()) {
    idx = static_cast<std::size_t>(std::distance(knots_.begin(), it) - 1);
  }
  if (idx >= segments_.size()) {
    idx = segments_.size() - 1;
  }
  u = clamped - knots_[idx];
  return idx;
}

State ParameterizedPath::position(double s) const
{
  double u = 0.0;
  const std::size_t k = findSegment(s, u);
  const auto & seg = segments_[k];
  State q(kDof);
  for (std::size_t j = 0; j < kDof; ++j) {
    q[j] = seg.a[j] + u * (seg.b[j] + u * (seg.c[j] + u * seg.d[j]));
  }
  return q;
}

State ParameterizedPath::firstDerivative(double s) const
{
  double u = 0.0;
  const std::size_t k = findSegment(s, u);
  const auto & seg = segments_[k];
  State dq(kDof);
  for (std::size_t j = 0; j < kDof; ++j) {
    dq[j] = seg.b[j] + u * (2.0 * seg.c[j] + 3.0 * seg.d[j] * u);
  }
  return dq;
}

State ParameterizedPath::secondDerivative(double s) const
{
  double u = 0.0;
  const std::size_t k = findSegment(s, u);
  const auto & seg = segments_[k];
  State d2q(kDof);
  for (std::size_t j = 0; j < kDof; ++j) {
    d2q[j] = 2.0 * seg.c[j] + 6.0 * seg.d[j] * u;
  }
  return d2q;
}

PathPostProcessor::PathPostProcessor(
  PostProcessConfig config,
  std::shared_ptr<const StateValidator> validator)
: config_(std::move(config)), validator_(std::move(validator))
{
  if (!validator_) {
    throw std::invalid_argument("PathPostProcessor requires a non-null StateValidator");
  }
  if (!std::isfinite(config_.max_path_deviation) || config_.max_path_deviation <= 0.0) {
    throw std::invalid_argument("max_path_deviation must be > 0");
  }
  if (!std::isfinite(config_.collision_check_resolution) ||
    config_.collision_check_resolution <= 0.0)
  {
    throw std::invalid_argument("collision_check_resolution must be > 0");
  }
}

Path PathPostProcessor::shortcut(const Path & raw_path, std::uint32_t seed) const
{
  if (raw_path.size() <= 2 || config_.shortcut_iterations == 0) {
    return raw_path;
  }
  Path path = raw_path;
  // 先頭と末尾が直接接続できるなら 2点に短縮する。
  if (validator_->isMotionValid(path.front(), path.back())) {
    return {path.front(), path.back()};
  }

  std::mt19937_64 rng(seed);
  for (std::size_t iter = 0; iter < config_.shortcut_iterations && path.size() > 2; ++iter) {
    std::uniform_int_distribution<std::size_t> dist(0, path.size() - 1);
    std::size_t i = dist(rng);
    std::size_t j = dist(rng);
    if (i > j) {
      std::swap(i, j);
    }
    if (j <= i + 1) {
      continue;
    }
    if (validator_->isMotionValid(path[i], path[j])) {
      path.erase(
        path.begin() + static_cast<std::ptrdiff_t>(i + 1),
        path.begin() + static_cast<std::ptrdiff_t>(j));
    }
  }
  return path;
}

double PathPostProcessor::maxDeviationFromPolyline(
  const ParameterizedPath & spline,
  const Path & polyline) const
{
  const std::vector<double> poly_knots = computeChordKnots(polyline);
  constexpr std::size_t kSamples = 200;
  double max_dev = 0.0;
  for (std::size_t i = 0; i <= kSamples; ++i) {
    const double s = static_cast<double>(i) / static_cast<double>(kSamples);
    const State qs = spline.position(s);
    const State qp = evaluatePolyline(polyline, poly_knots, s);
    max_dev = std::max(max_dev, euclideanDistance(qs, qp));
  }
  return max_dev;
}

bool PathPostProcessor::isSplineCollisionFree(const ParameterizedPath & spline) const
{
  if (spline.isStationary()) {
    return validator_->isStateValid(spline.position(0.0));
  }
  // 経路全体の導関数ノルムを見積もり、各ステップの関節角差が resolution 以下になるよう分割する。
  double max_deriv = 1.0;
  for (int i = 0; i <= 50; ++i) {
    const double s = static_cast<double>(i) / 50.0;
    const State dq = spline.firstDerivative(s);
    for (double v : dq) {
      max_deriv = std::max(max_deriv, std::abs(v));
    }
  }
  const std::size_t steps = std::max<std::size_t>(
    50, static_cast<std::size_t>(std::ceil(max_deriv / config_.collision_check_resolution)) * 2);

  State prev = spline.position(0.0);
  if (!validator_->isStateValid(prev)) {
    return false;
  }
  for (std::size_t k = 1; k <= steps; ++k) {
    const double s = static_cast<double>(k) / static_cast<double>(steps);
    const State cur = spline.position(s);
    if (!validator_->isStateValid(cur)) {
      return false;
    }
    if (maxAbsDiff(prev, cur) > config_.collision_check_resolution) {
      if (!validator_->isMotionValid(prev, cur)) {
        return false;
      }
    }
    prev = cur;
  }
  return true;
}

PostProcessResult PathPostProcessor::process(const Path & raw_path, std::uint32_t seed) const
{
  PostProcessResult result;
  if (raw_path.size() < 2) {
    result.failure = FailureCode::kInvalidInput;
    result.message = "raw_path must have at least 2 states";
    return result;
  }
  for (std::size_t i = 0; i < raw_path.size(); ++i) {
    if (!validator_->isStateValid(raw_path[i])) {
      result.failure = FailureCode::kCollisionDetected;
      result.message = "raw_path contains an invalid waypoint";
      return result;
    }
    if (i + 1 < raw_path.size() && !validator_->isMotionValid(raw_path[i], raw_path[i + 1])) {
      result.failure = FailureCode::kCollisionDetected;
      result.message = "raw_path contains a colliding edge";
      return result;
    }
  }

  result.shortcut_path = deduplicateWaypoints(shortcut(raw_path, seed));
  if (result.shortcut_path.size() == 2 &&
    euclideanDistance(result.shortcut_path.front(), result.shortcut_path.back()) <= 1e-12)
  {
    result.spline = ParameterizedPath::makeStationary(result.shortcut_path.front());
    return result;
  }

  // 保守的な再試行: 偏差超過や衝突が起きた場合は折れ線辺を 2^retry 分割して節点を増やし、
  // 元折れ線へ漸近させる（仕様書 第8.2節）。
  std::size_t subdivisions = 1;
  for (std::size_t retry = 0; retry <= config_.max_smoothing_retries; ++retry) {
    const Path refined = subdividePolyline(result.shortcut_path, subdivisions);
    ParameterizedPath candidate = ParameterizedPath::fromWaypoints(refined);
    const double dev = maxDeviationFromPolyline(candidate, result.shortcut_path);
    if (dev <= config_.max_path_deviation && isSplineCollisionFree(candidate)) {
      result.spline = std::move(candidate);
      result.max_deviation = dev;
      result.smoothing_retries_used = retry;
      return result;
    }
    subdivisions *= 2;
  }

  result.failure = FailureCode::kSmoothingFailed;
  result.message = "spline smoothing exceeded deviation or collided after max retries";
  return result;
}

}  // namespace path_planner
