#ifndef NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_
#define NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace rotation_constraints
{

constexpr double kTurn = 6.28318530717958647692;
constexpr double kTolerance = 1e-6;

inline bool legal(double angle)
{
  return std::isfinite(angle) && angle >= -kTurn - kTolerance &&
         angle <= kTolerance;
}

inline double clamp(double angle)
{
  return std::max(-kTurn, std::min(0.0, angle));
}

inline std::vector<double> candidates(double angle)
{
  if (!std::isfinite(angle)) {
    return {};
  }
  double normalized = std::fmod(angle, kTurn);
  if (normalized > 0.0) {normalized -= kTurn;}
  if (std::abs(normalized) <= kTolerance ||
    std::abs(normalized + kTurn) <= kTolerance)
  {
    return {-kTurn, 0.0};
  }
  return {normalized};
}

inline bool solve(
  double start, const std::vector<double> & raw_angles,
  int direction, std::vector<double> & angles)
{
  angles.clear();
  if (!legal(start) || (direction != 1 && direction != -1)) {return false;}
  double previous = clamp(start);
  for (double raw : raw_angles) {
    auto options = candidates(raw);
    if (direction < 0) {std::reverse(options.begin(), options.end());}
    bool found = false;
    for (double candidate : options) {
      if (direction * (candidate - previous) < -kTolerance) {continue;}
      if (direction * (candidate - previous) < 0.0) {candidate = previous;}
      angles.push_back(candidate);
      previous = candidate;
      found = true;
      break;
    }
    if (!found) {
      angles.clear();
      return false;
    }
  }
  return true;
}

inline bool continuous_base_segment(double x0, double y0, double x1, double y1)
{
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) ||
    !std::isfinite(y1))
  {
    return false;
  }
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (x0 * dx + y0 * dy);
  const double nearest = a > 0.0 ? std::clamp(-b / (2.0 * a), 0.0, 1.0) : 0.0;
  const double nx = x0 + nearest * dx;
  const double ny = y0 + nearest * dy;
  // The base axis and the principal-atan2 branch need separate joint planning.
  if (nx * nx + ny * ny < 1e-6 ||
    std::abs(std::atan2(x1, y1) - std::atan2(x0, y0)) > kTurn / 2.0)
  {
    return false;
  }
  return true;
}

// Validate the actual linear XYZ/phi interpolation between route samples.
inline bool legal_phi_segment(
  double x0, double y0, double x1, double y1, double phi0, double phi1,
  int direction = 0)
{
  if (!continuous_base_segment(x0, y0, x1, y1) ||
    !std::isfinite(phi0) || !std::isfinite(phi1) ||
    direction < -1 || direction > 1) {return false;}
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (x0 * dx + y0 * dy);
  const double c = x0 * x0 + y0 * y0;
  const double delta = phi1 - phi0;
  if (direction != 0) {
    // The derivative extrema occur at the extrema of the squared base radius.
    const double nearest = a > 0.0 ? std::clamp(-b / (2.0 * a), 0.0, 1.0) : 0.0;
    for (double t : {0.0, nearest, 1.0}) {
      const double x = x0 + t * dx;
      const double y = y0 + t * dy;
      const double wrist_derivative = delta - (y0 * dx - x0 * dy) / (x*x + y*y);
      if (direction * wrist_derivative < -kTolerance) {return false;}
    }
  }
  const auto check = [&](double t) {
      return legal(phi0 + t * delta - std::atan2(x0 + t * dx, y0 + t * dy));
    };
  if (!check(0.0) || !check(1.0)) {return false;}
  if (a == 0.0 || delta == 0.0) {return true;}
  // d(phi-q0)/dt = delta - (y0*dx-x0*dy)/r(t)^2.
  const double stationary_c = c - (y0 * dx - x0 * dy) / delta;
  const double discriminant = b * b - 4.0 * a * stationary_c;
  if (discriminant < 0.0) {return true;}
  const double root = std::sqrt(discriminant);
  for (double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
    if (t > 0.0 && t < 1.0 && !check(t)) {return false;}
  }
  return true;
}

// Include interior extrema when XYZ and the wrist are linearly interpolated.
inline double wrist_segment_phi_travel(
  double x0, double y0, double x1, double y1, double wrist0, double wrist1)
{
  if (!continuous_base_segment(x0, y0, x1, y1) || !legal(wrist0) || !legal(wrist1)) {
    return std::numeric_limits<double>::infinity();
  }
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double delta = wrist1 - wrist0;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (x0 * dx + y0 * dy);
  std::vector<double> times{0.0, 1.0};
  if (a > 0.0 && delta != 0.0) {
    const double c = x0*x0 + y0*y0 + (y0*dx - x0*dy) / delta;
    const double discriminant = b*b - 4.0*a*c;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      for (double t : {(-b-root)/(2.0*a), (-b+root)/(2.0*a)}) {
        if (t > 0.0 && t < 1.0) {times.push_back(t);}
      }
    }
  }
  std::sort(times.begin(), times.end());
  double previous = std::atan2(x0, y0) + wrist0;
  double travel = 0.0;
  for (size_t i = 1; i < times.size(); ++i) {
    const double t = times[i];
    const double phi = std::atan2(x0 + t*dx, y0 + t*dy) + wrist0 + t*delta;
    travel += std::abs(phi - previous);
    previous = phi;
  }
  return travel;
}

// At most two wrist endpoints exist at each target; retain both for lookahead.
template<typename EdgeLegal>
inline bool solve_minimum_phi(
  double start_phi, const std::vector<double> & bases,
  const std::vector<double> & raw_angles, EdgeLegal edge_legal,
  std::vector<double> & angles, double & travel)
{
  angles.clear();
  travel = 0.0;
  if (!std::isfinite(start_phi) || bases.size() != raw_angles.size() || bases.empty()) {
    return false;
  }
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> options;
  std::vector<std::array<int, 2>> parents;
  std::array<double, 2> previous_cost{0.0, infinity};
  std::vector<double> previous_phi{start_phi};
  for (size_t i = 0; i < bases.size(); ++i) {
    if (!std::isfinite(bases[i])) {return false;}
    options.push_back(candidates(raw_angles[i]));
    parents.push_back({-1, -1});
    std::array<double, 2> costs{infinity, infinity};
    std::vector<double> phis;
    for (size_t target = 0; target < options.back().size(); ++target) {
      const double phi = bases[i] + options.back()[target];
      phis.push_back(phi);
      for (size_t previous = 0; previous < previous_phi.size(); ++previous) {
        const double cost = previous_cost[previous] + std::abs(phi - previous_phi[previous]);
        if (cost < costs[target] && edge_legal(i, previous_phi[previous], phi)) {
          costs[target] = cost;
          parents.back()[target] = static_cast<int>(previous);
        }
      }
    }
    previous_cost = costs;
    previous_phi = std::move(phis);
  }
  size_t selected = previous_cost[0] <= previous_cost[1] ? 0 : 1;
  if (!std::isfinite(previous_cost[selected])) {return false;}
  travel = previous_cost[selected];
  angles.resize(options.size());
  for (size_t i = options.size(); i-- > 0;) {
    angles[i] = options[i][selected];
    selected = static_cast<size_t>(parents[i][selected]);
  }
  return true;
}

}  // namespace rotation_constraints

#endif  // NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_
