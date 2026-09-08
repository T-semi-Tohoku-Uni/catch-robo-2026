#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav_director/route_planner.hpp"

#if defined(X) || defined(Y) || defined(PI)
#error "The public planner header must not expose kinematics macros"
#endif

namespace
{

constexpr double turn = 6.28318530717958647692;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

geometry_msgs::msg::Pose target(const nav_director::Point3D & point)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x / 1000.0;
  pose.position.y = point.y / 1000.0;
  pose.position.z = point.z / 1000.0;
  pose.orientation.z = std::sin(point.phi / 2.0);
  pose.orientation.w = std::cos(point.phi / 2.0);
  return pose;
}

nav_director::PlannerState state(
  nav_director::RoutePlanner & planner, const nav_director::Point3D & point,
  double wrist)
{
  std::array<float, 4> joints{};
  require(planner.solveIK(point, joints), "Fixture must be reachable");
  joints[3] = static_cast<float>(wrist);
  return planner.stateFromJoints(joints);
}

void check()
{
  nav_director::RoutePlanner planner;
  const nav_director::Point3D a{675.0, 200.0, 300.0, 0.0};
  const nav_director::Point3D b{670.0, -110.0, 220.0, 0.0};
  const auto zero = state(planner, a, 0.0);
  const auto wound = state(planner, a, -turn);
  require(std::abs(zero.pose.x - wound.pose.x) < 1e-6, "Winding changed XYZ");
  require(std::abs(zero.wrist - wound.wrist - turn) < 1e-6, "Winding was lost");

  const auto route = planner.generateRoute(zero, {b});
  require(route.size() == 201, "Normal route sampling changed");
  require(std::abs(route.front().x - zero.pose.x) < 1e-8, "Start was not included");
  require(std::abs(route.back().y - b.y) < 1e-8, "End changed");
  const auto inline_route = planner.generateRoute(zero, {{700, 100, 300, 0}, b});
  require(inline_route.size() == 351, "Waypoint densification changed");

  nav_director::PlanRotationGroup::Request request;
  request.targets = {target(b)};
  request.route_ends = {1};
  request.allow_wrist_reversal = true;
  request.limit_phi_travel = true;
  request.max_phi_travel = 0.17453292519943295;
  const auto rejected = planner.planGroup(zero, request);
  require(!rejected.success, "Unwound A to B must exceed the small phi budget");
  require(rejected.message.find("exceeds") != std::string::npos,
    "The phi budget failure was not explained");
  const auto accepted = planner.planGroup(wound, request);
  require(accepted.success, accepted.message);
  require(accepted.phi_travel < 1e-5, "Wound A to B should preserve phi");
  require(accepted.routes.size() == 1, "Route count changed");
  require(accepted.routes[0].path.header.stamp.sec == 0 &&
    accepted.routes[0].path.header.stamp.nanosec == 0, "Offline planner used a clock");
  require(std::abs(accepted.routes[0].wrist_angles.back() + 6.220766497) < 1e-5,
    "Ending wrist angle changed");

  const nav_director::Point3D pick{375.0, 238.0, 196.95, -turn / 2.0};
  std::array<float, 4> pick_joints{};
  require(planner.solveIK(pick, pick_joints), "Pick fixture is unreachable");
  const auto pick_state = planner.stateFromJoints(pick_joints);
  request.targets = {target(a), target(b)};
  request.route_ends = {1, 2};
  request.allow_wrist_reversal = false;
  request.limit_phi_travel = false;
  const auto direction = planner.planGroup(pick_state, request);
  require(!direction.success && direction.message.find("No single wrist direction") == 0,
    "Legacy ending must reject the direction reversal");

  request.allow_wrist_reversal = true;
  request.limit_phi_travel = true;
  request.max_phi_travel = turn / 2.0 + 0.01;
  const auto lookahead = planner.planGroup(pick_state, request);
  require(lookahead.success, lookahead.message);
  require(lookahead.routes.size() == 2, "Lookahead route count changed");
  require(std::abs(lookahead.routes[0].wrist_angles.back() + turn) < 1e-5,
    "Lookahead did not select the useful winding at A");

  std::array<float, 4> joints{};
  require(!planner.solveIK({10000, 10000, 10000, 0}, joints),
    "An unreachable position was accepted");
  bool invalid_rejected = false;
  try {
    planner.generateRoute(zero, {{0, std::numeric_limits<double>::quiet_NaN(), 0, 0}});
  } catch (const std::runtime_error &) {
    invalid_rejected = true;
  }
  require(invalid_rejected, "Non-finite route was accepted");
  invalid_rejected = false;
  try {
    planner.stateFromJoints({0, 0, 0, std::numeric_limits<float>::quiet_NaN()});
  } catch (const std::runtime_error &) {
    invalid_rejected = true;
  }
  require(invalid_rejected, "Non-finite starting state was accepted");
}

}  // namespace

int main()
{
  try {
    check();
    std::cout << "Offline route planner checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
