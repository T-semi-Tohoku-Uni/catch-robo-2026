#ifndef NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_
#define NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_

#include <algorithm>
#include <cmath>
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

}  // namespace rotation_constraints

#endif  // NAV_DIRECTOR_ROTATION_CONSTRAINTS_HPP_
