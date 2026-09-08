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
  expect(!solve(-3.0, {std::numeric_limits<double>::quiet_NaN()}, 1, angles),
        "Non-finite targets must fail");
  expect(!solve(std::numeric_limits<double>::infinity(), {-2.0}, 1, angles),
        "Non-finite initial wrist must fail");
  expect(!solve(-3.0, {-2.0}, 0, angles), "Invalid direction must fail");
  expect(solve(-1.0, {-1.0 - kTolerance / 2}, 1, angles) && angles.back() == -1.0,
        "Float noise must not reverse the planned wrist");
  std::cout << "17 rotation constraint checks passed\n";
}
