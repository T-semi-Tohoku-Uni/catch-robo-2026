#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include "path_planner/robot_model.hpp"
#include "ros2_inverse_kinematics/robot_kinematics.h"

namespace
{
void check(const std::array<float, 4> & angles, robot_kinematics & production)
{
  auto mutable_angles = angles;
  float pose[6]{};
  production.forward_kinematics(pose, mutable_angles.data());
  path_planner::State q(angles.begin(), angles.end());
  const auto frames = path_planner::forwardKinematics(
    path_planner::defaultKinematicChain(), path_planner::toUrdfJointPositions(q));
  const auto & flange = frames[5];
  const double xyz[3] = {
    flange.translation.x, flange.translation.y, flange.translation.z};
  for (int i = 0; i < 3; ++i) {
    // The production API rounds millimetres to float.
    if (std::abs(xyz[i] - pose[i] / 1000.0) > 2e-7) {
      throw std::runtime_error("Preview position differs from production FK");
    }
  }
  const double phi = static_cast<double>(pose[PHI]);
  const double yaw = std::atan2(flange.rotation(1, 0), flange.rotation(0, 0));
  if (std::abs(std::remainder(yaw - phi, 2.0 * path_planner::kPi)) > 2e-6) {
    throw std::runtime_error("Preview yaw differs from production phi");
  }
}
}  // namespace

int main()
{
  try {
    robot_kinematics production;
    const std::array<std::array<float, 4>, 5> anchors{{
      {{0.0F, 0.0F, 0.0F, 0.0F}},
      {{-0.7582F, 0.7605F, 2.1702F, -0.8126F}},
      {{0.7582F, 0.7605F, 2.1702F, -2.3290F}},
      {{1.57F, -0.4F, 1.8F, -6.283185F}},
      {{-1.57F, 1.0F, 2.0F, 0.0F}},
    }};
    for (const auto & q : anchors) {
      check(q, production);
    }
    std::mt19937 random(20261022);
    std::uniform_real_distribution<float> angle(-3.1415926F, 3.1415926F);
    for (int i = 0; i < 200; ++i) {
      check({angle(random), angle(random), angle(random)}, production);
    }
    std::cout << "205 preview/production FK position and yaw comparisons passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
