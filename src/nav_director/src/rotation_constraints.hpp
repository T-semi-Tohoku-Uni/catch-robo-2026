#ifndef NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_
#define NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "ros2_inverse_kinematics/homogeneous_transform.h"

namespace rotation_constraints
{

constexpr double kTurn = 6.28318530717958647692;
constexpr double kTolerance = 1e-6;

inline bool legal(double angle)
{
  return std::isfinite(angle) && angle >= -kTurn - kTolerance &&
         angle <= kTolerance;
}

// Measurement tolerance never expands the range of planned or commanded angles.
inline bool legal_feedback(double angle, double tolerance_rad)
{
  return std::isfinite(angle) && std::isfinite(tolerance_rad) &&
         tolerance_rad >= 0.0 && tolerance_rad < kTurn / 2.0 &&
         angle >= -kTurn - tolerance_rad - kTolerance &&
         angle <= tolerance_rad + kTolerance;
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
    std::abs(catchrobo_kinematics::base_angle(x1, y1) -
    catchrobo_kinematics::base_angle(x0, y0)) > kTurn / 2.0)
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
  const double cross = y0 * dx - x0 * dy;
  if (direction != 0) {
    // The derivative extrema occur at the extrema of the squared base radius.
    const double nearest = a > 0.0 ? std::clamp(-b / (2.0 * a), 0.0, 1.0) : 0.0;
    for (double t : {0.0, nearest, 1.0}) {
      const double x = x0 + t * dx;
      const double y = y0 + t * dy;
      const double wrist_derivative = delta - cross / (x*x + y*y);
      if (direction * wrist_derivative < -kTolerance) {return false;}
    }
  }
  const auto check = [&](double t) {
      return legal(phi0 + t * delta - catchrobo_kinematics::base_angle(
        x0 + t * dx, y0 + t * dy));
    };
  if (!check(0.0) || !check(1.0)) {return false;}
  if (a == 0.0 || delta == 0.0) {return true;}
  // d(phi-q0)/dt = delta - cross/r(t)^2.
  const double stationary_c = c - cross / delta;
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
  const double cross = y0 * dx - x0 * dy;
  std::vector<double> times{0.0, 1.0};
  if (a > 0.0 && delta != 0.0) {
    const double c = x0*x0 + y0*y0 + cross / delta;
    const double discriminant = b*b - 4.0*a*c;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      for (double t : {(-b-root)/(2.0*a), (-b+root)/(2.0*a)}) {
        if (t > 0.0 && t < 1.0) {times.push_back(t);}
      }
    }
  }
  std::sort(times.begin(), times.end());
  double previous = catchrobo_kinematics::base_angle(x0, y0) + wrist0;
  double travel = 0.0;
  for (size_t i = 1; i < times.size(); ++i) {
    const double t = times[i];
    const double phi = catchrobo_kinematics::base_angle(
      x0 + t*dx, y0 + t*dy) + wrist0 + t*delta;
    travel += std::abs(phi - previous);
    previous = phi;
  }
  return travel;
}

struct PhiTravelInterval
{
  size_t start_target, end_target;
  double max_phi_travel;
};

// Keep nondominated total/local costs so later bounds can select an earlier winding.
template<typename EdgeCost>
inline bool solve_minimum_phi_cost(
  double start_phi, const std::vector<double> & bases,
  const std::vector<double> & raw_angles, EdgeCost edge_cost,
  std::vector<double> & angles, double & travel,
  const std::vector<PhiTravelInterval> & intervals = {})
{
  angles.clear();
  travel = 0.0;
  if (!std::isfinite(start_phi) || bases.size() != raw_angles.size() || bases.empty()) {
    return false;
  }
  size_t previous_end = 0;
  for (const auto & interval : intervals) {
    if (interval.start_target < previous_end || interval.start_target >= interval.end_target ||
      interval.end_target > bases.size() || !std::isfinite(interval.max_phi_travel) ||
      interval.max_phi_travel < 0.0) {return false;}
    previous_end = interval.end_target;
  }
  struct Label {double phi, wrist, cost, local; size_t parent;};
  std::vector<std::vector<Label>> layers{{{start_phi, 0.0, 0.0, 0.0, 0}}};
  size_t interval_index = 0;
  for (size_t i = 0; i < bases.size(); ++i) {
    if (!std::isfinite(bases[i])) {return false;}
    while (interval_index < intervals.size() && i >= intervals[interval_index].end_target) {
      ++interval_index;
    }
    const auto * interval = interval_index < intervals.size() &&
      i >= intervals[interval_index].start_target ? &intervals[interval_index] : nullptr;
    const auto & previous = layers.back();
    std::vector<Label> next;
    for (const double wrist : candidates(raw_angles[i])) {
      const double phi = bases[i] + wrist;
      if (!std::isfinite(phi)) {return false;}
      for (size_t parent = 0; parent < previous.size(); ++parent) {
        const auto & before = previous[parent];
        const double amount = edge_cost(i, before.phi, phi);
        if (!std::isfinite(amount) || amount < 0.0) {continue;}
        const double local = interval ? amount +
          (i == interval->start_target ? 0.0 : before.local) : 0.0;
        if (interval && local > interval->max_phi_travel + kTolerance) {continue;}
        const double cost = before.cost + amount;
        if (!std::isfinite(cost) || !std::isfinite(local)) {continue;}
        const auto dominates = [wrist, cost, local](const Label & label) {
            return label.wrist == wrist && label.cost <= cost && label.local <= local;
          };
        if (std::any_of(next.begin(), next.end(), dominates)) {continue;}
        next.erase(std::remove_if(next.begin(), next.end(),
          [wrist, cost, local](const Label & label) {
            return label.wrist == wrist && cost <= label.cost && local <= label.local;
          }), next.end());
        next.push_back({phi, wrist, cost, local, parent});
      }
    }
    if (next.empty()) {return false;}
    layers.push_back(std::move(next));
  }
  const auto & last = layers.back();
  size_t selected = static_cast<size_t>(std::min_element(last.begin(), last.end(),
    [](const Label & a, const Label & b) {return a.cost < b.cost;}) - last.begin());
  travel = last[selected].cost;
  angles.resize(bases.size());
  for (size_t i = bases.size(); i > 0; --i) {
    const auto & label = layers[i][selected];
    angles[i - 1] = label.wrist;
    selected = label.parent;
  }
  return true;
}

template<typename EdgeLegal>
inline bool solve_minimum_phi(
  double start_phi, const std::vector<double> & bases,
  const std::vector<double> & raw_angles, EdgeLegal edge_legal,
  std::vector<double> & angles, double & travel,
  const std::vector<PhiTravelInterval> & intervals = {})
{
  return solve_minimum_phi_cost(start_phi, bases, raw_angles,
    [&edge_legal](size_t i, double phi0, double phi1) {
      return edge_legal(i, phi0, phi1) ? std::abs(phi1 - phi0) :
             std::numeric_limits<double>::infinity();
    }, angles, travel, intervals);
}

}  // namespace rotation_constraints

#endif  // NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_
