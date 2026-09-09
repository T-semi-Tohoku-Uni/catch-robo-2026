#include "../src/rotation_constraints.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
void expect(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // namespace

int main()
{
  using namespace rotation_constraints;
  std::vector<double> angles;
  expect(candidates(0.0).size() == 2, "Zero must allow both wrist endpoints");
  expect(candidates(kTurn).size() == 2, "Full turn must allow both wrist endpoints");
  expect(candidates(-kTurn).size() == 2, "Negative turn must allow both endpoints");
  expect(solve(-3.0, {-2.0, -1.0, 0.0}, 1, angles) && angles.back() == 0.0,
        "Increasing route must reach zero without wrapping");
  expect(!solve(-3.0, {-2.0, -1.0, 0.0}, -1, angles),
        "Decreasing route must reject increasing interior targets");
  expect(solve(-3.0, {-4.0, -5.0, 0.0}, -1, angles) && angles.back() == -kTurn,
        "Decreasing route must use the negative full-turn endpoint");
  expect(!solve(-3.0, {-4.0, -5.0, 0.0}, 1, angles),
        "Increasing route must reject decreasing interior targets");
  expect(!solve(-3.0, {-4.0, -2.0}, 1, angles) &&
        !solve(-3.0, {-4.0, -2.0}, -1, angles),
        "Nonmonotonic group must be rejected in both directions");
  expect(solve(0.0, {-0.5, 0.0}, -1, angles) && angles.back() == -kTurn,
        "Endpoint selection must preserve a required full turn");
  expect(solve(-kTurn, {-5.0, 0.0}, 1, angles) && angles.back() == 0.0,
        "Increasing endpoint selection must preserve a required full turn");
  expect(solve(-kTurn, {0.0}, -1, angles) && angles.back() == -kTurn,
        "Equivalent endpoint must not force an unnecessary turn");
  expect(legal(static_cast<float>(-kTurn)), "Float32 feedback must allow the lower limit");
  expect(!legal(-kTurn - 0.001) && !legal(0.001), "Out-of-limit wrists must be rejected");
  const double feedback_tolerance = kTurn / 60.0;
  for (const double boundary : {0.0, -kTurn}) {
    const double sign = boundary == 0.0 ? 1.0 : -1.0;
    expect(legal_feedback(boundary + sign * feedback_tolerance, feedback_tolerance),
          "Measured wrists at the configured six-degree boundary must be accepted");
    expect(!legal_feedback(boundary + sign * (feedback_tolerance + 0.001), feedback_tolerance),
          "Measured wrists beyond the configured tolerance must be rejected");
    expect(!legal(boundary + sign * feedback_tolerance),
          "Measurement tolerance must not expand command limits");
    expect(!legal_feedback(boundary + sign * 0.001, 0.0),
          "Zero feedback tolerance must restore the numeric-only limit");
  }
  for (const double invalid : {-0.1, kTurn / 2.0,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    expect(!legal_feedback(-1.0, invalid), "Invalid feedback tolerances must fail closed");
    expect(!legal_feedback(invalid == -0.1 ? 1.0 : invalid, feedback_tolerance),
          "Invalid measured angles must not be accepted");
  }
  expect(!solve(-3.0, {std::numeric_limits<double>::quiet_NaN()}, 1, angles),
        "Non-finite targets must fail");
  expect(!solve(std::numeric_limits<double>::infinity(), {-2.0}, 1, angles),
        "Non-finite initial wrist must fail");
  expect(!solve(-3.0, {-2.0}, 0, angles), "Invalid direction must fail");
  expect(solve(-1.0, {-1.0 - kTolerance / 2}, 1, angles) && angles.back() == -1.0,
        "Float noise must not reverse the planned wrist");
  expect(legal_phi_segment(0.0, 390.0, -5.0, 80.0, -kTurn, -kTurn),
        "Ending A to B must permit constant unwrapped phi at the lower wrist branch");
  expect(legal_phi_segment(0.0, 390.0, -5.0, 80.0, -kTurn, -kTurn, 1),
        "A zero-phi-travel ending must also satisfy increasing wrist direction");
  expect(!legal_phi_segment(0.0, 390.0, -5.0, 80.0, -kTurn, -kTurn, -1),
        "A zero-phi-travel ending must reject the opposite wrist direction");
  expect(!legal_phi_segment(0.0, 390.0, -5.0, 80.0, 0.0, 0.0),
        "Ending A at wrist zero cannot retain phi zero on the way to B");
  expect(!legal_phi_segment(-1.0, 1.0, -10.0, 1.0,
          std::atan2(-1.0, 1.0), std::atan2(-10.0, 1.0)),
        "Interior wrist extrema must reject an upper-limit excursion");
  expect(!legal_phi_segment(1.0, 1.0, 10.0, 1.0,
          std::atan2(1.0, 1.0) - kTurn, std::atan2(10.0, 1.0) - kTurn),
        "Interior wrist extrema must reject a lower-limit excursion");
  expect(!continuous_base_segment(-1.0, -10.0, 1.0, -10.0),
        "The raw base atan2 discontinuity must not be hidden by unwrapping");
  expect(!continuous_base_segment(-1.0, 0.0, 1.0, 0.0),
        "A base-axis crossing between samples must be rejected");
  expect(legal_phi_segment(-1.0, 1.0, 1.0, 1.0, -4.0, -2.3) &&
          !legal_phi_segment(-1.0, 1.0, 1.0, 1.0, -4.0, -2.3, 1),
        "Direction checks must include the closest radius between samples");
  const double ending_base = std::atan2(-5.0, 80.0);
  const double ending_amount = wrist_segment_phi_travel(
      0.0, 390.0, -5.0, 80.0, -kTurn, -kTurn - ending_base);
  expect(std::abs(ending_amount - 0.04696155549) < 1e-9,
        "Wrist-linear ending travel must include its interior phi reversal");
  expect(std::abs(wrist_segment_phi_travel(0.0, 1.0, 0.0, 1.0, 0.0, -kTurn) -
          kTurn) < 1e-12,
        "A full turn must not disappear because start and end orientations match");
  double travel = 0.0;
  for (int direction : {0, 1}) {
    const auto ending_direction = [direction](size_t, double phi0, double phi1) {
        return legal_phi_segment(0.0, 390.0, -5.0, 80.0, phi0, phi1, direction);
      };
    expect(solve_minimum_phi(-kTurn, {ending_base}, {-ending_base},
            ending_direction, angles, travel) && travel == 0.0,
          "AB must meet a zero phi budget with or without increasing wrist direction");
  }
  const auto any_edge = [](size_t, double, double) {return true;};
  expect(!solve_minimum_phi_cost(0.0, {0.0}, {0.0},
          [](size_t, double, double) {return 1.0;}, angles, travel, {{0, 1, 0.8}}),
        "A custom interpolation must count interior travel despite identical endpoints");
  expect(solve_minimum_phi(0.0, {0.0, 0.0, 0.0}, {0.0, -4.0, -2.0},
          any_edge, angles, travel, {{1, 3, 5.0}}) && angles.front() == -kTurn &&
          std::abs(travel - (2.0 * kTurn - 2.0)) < 1e-10,
        "Keep a more expensive prefix when it preserves the local budget after a merge");
  expect(!solve_minimum_phi(0.0, {0.0, 0.0, 0.0}, {0.0, -4.0, -2.0},
          any_edge, angles, travel, {{1, 3, 4.0}}),
        "Reject all winding paths exceeding a cumulative subinterval budget");
  expect(solve_minimum_phi(-2.0, {0.0, 0.0, 0.0}, {-2.0, -2.4, -2.0},
          any_edge, angles, travel, {{0, 1, 0.0}, {1, 2, 0.4}, {2, 3, 0.4}}),
        "Reset each adjacent subinterval without resetting the global total");
  expect(std::abs(travel - 0.8) < 1e-10, "Adjacent budgets must not erase global travel");
  expect(solve_minimum_phi(-kTurn / 2.0, {0.0, ending_base},
          {0.0, -ending_base}, any_edge, angles, travel) &&
          angles[0] == -kTurn && std::abs(travel - kTurn / 2.0) < 1e-12,
        "Endpoint selection must look ahead through A to B instead of choosing nearest A");
  const auto ending_edges = [](size_t index, double phi0, double phi1) {
      return index == 0 ? legal_phi_segment(-300.0, 428.0, 0.0, 390.0, phi0, phi1) :
             legal_phi_segment(0.0, 390.0, -5.0, 80.0, phi0, phi1);
    };
  expect(solve_minimum_phi(-kTurn / 2.0, {0.0, ending_base},
          {0.0, -ending_base}, ending_edges, angles, travel) &&
          angles[0] == -kTurn && std::abs(travel - kTurn / 2.0) < 1e-12,
        "Configured PICK retreat through ending must choose the legal constant-phi AB branch");
  expect(solve_minimum_phi(0.0, {0.0, 0.0}, {0.0, 0.0},
          any_edge, angles, travel) && travel == 0.0 && angles[0] == 0.0,
        "A feasible zero-phi-travel plan must remain exactly zero");
  const auto reject_short = [](size_t index, double, double phi) {
      return index != 0 || phi == 0.0;
    };
  expect(solve_minimum_phi(-kTurn / 2.0, {0.0, ending_base},
          {0.0, -ending_base}, reject_short, angles, travel) &&
          angles[0] == 0.0 && std::abs(travel - 1.5 * kTurn) < 1e-12,
        "A geometrically infeasible short branch must not hide a longer legal branch");
  const auto reject_all = [](size_t, double, double) {return false;};
  expect(!solve_minimum_phi(-1.0, {0.0}, {-2.0}, reject_all, angles, travel),
        "An infeasible phi graph must return no partial plan");
  expect(!solve_minimum_phi(-1.0, {}, {-2.0}, any_edge, angles, travel),
        "Mismatched phi targets must be rejected");
  const double fallback_base0 = std::atan2(-100.0, 100.0);
  const double fallback_base1 = std::atan2(-400.0, 100.0);
  for (int direction : {1, -1}) {
    const auto phi_direction = [direction](size_t, double phi0, double phi1) {
        return legal_phi_segment(-100.0, 100.0, -400.0, 100.0, phi0, phi1, direction);
      };
    expect(!solve_minimum_phi(fallback_base0 - 3.0, {fallback_base1}, {-3.0},
            phi_direction, angles, travel),
          "Phi-linear interpolation must reject an interior wrist reversal");
    expect(solve(-3.0, {-3.0}, direction, angles) &&
            wrist_segment_phi_travel(-100.0, 100.0, -400.0, 100.0, -3.0, -3.0) < 0.6,
          "A rejected phi interpolation must retain the legal wrist-linear fallback");
  }
  std::vector<double> many(10000, 0.0);
  size_t edge_checks = 0;
  const auto count_edges = [&edge_checks](size_t, double, double) {
      ++edge_checks;
      return true;
    };
  expect(solve_minimum_phi(0.0, many, many, count_edges, angles, travel) &&
          angles.size() == many.size() && travel == 0.0 && edge_checks <= 4 * many.size(),
        "Endpoint lookahead must stay linear for 10000 ambiguous targets");
  std::cout << "Rotation constraint checks passed\n";
}
