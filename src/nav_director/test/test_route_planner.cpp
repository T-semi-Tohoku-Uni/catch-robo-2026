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

void checkFeedbackTolerance()
{
  nav_director::RoutePlanner planner;
  const nav_director::Point3D fixture{700.0, 200.0, 300.0, 0.0};
  const double tolerance = 6.0 * turn / 360.0;
  for (const double boundary : {0.0, -turn}) {
    const double outward = boundary == 0.0 ? 1.0 : -1.0;
    for (const bool reversal : {false, true}) {
      const auto measured = state(planner, fixture, boundary + outward * 5.9 * turn / 360.0);
      const auto original = measured;
      require(std::abs(measured.wrist - boundary) > 0.1,
        "Raw feedback must retain the measured overshoot");
      nav_director::PlanRotationGroup::Request request;
      request.targets = {target({fixture.x, fixture.y, fixture.z + 10.0,
        measured.base + boundary - outward * 0.2})};
      request.route_ends = {1};
      request.allow_wrist_reversal = reversal;
      const auto accepted = planner.planGroup(measured, request, tolerance);
      require(accepted.success, "5.9 degree feedback was rejected: " + accepted.message);
      require(accepted.routes.size() == 1, "Tolerance changed the route count");
      const auto & route = accepted.routes.front();
      require(std::abs(route.wrist_angles.front() - boundary) < 1e-8,
        "The planning start was wrapped instead of clamped to its nearest boundary");
      for (size_t i = 0; i < route.path.poses.size(); ++i) {
        const double wrist = route.wrist_angles.at(i);
        require(wrist >= -turn && wrist <= 0.0, "A planned command exceeded the strict wrist range");
        const auto & pose = route.path.poses[i].pose;
        const auto & q = pose.orientation;
        const double phi = 2.0 * std::atan2(q.z, q.w);
        std::array<float, 4> joints{};
        require(planner.solveIK({pose.position.x * 1000.0, pose.position.y * 1000.0,
          pose.position.z * 1000.0, phi}, joints), "Tolerated feedback produced invalid IK");
        require(std::abs(std::remainder(phi - joints[0] - wrist, turn)) < 1e-5,
          "Planned pose and bounded wrist became inconsistent");
        if (!route.phi_angles.empty()) {
          require(std::abs(route.phi_angles.at(i) - joints[0] - wrist) < 1e-5,
            "Unwrapped phi lost the physical wrist turn");
        }
      }
      if (!route.phi_angles.empty()) {
        require(std::abs(route.phi_angles.front() - measured.base - boundary) < 1e-5,
          "Planning phi did not follow the clamped start wrist");
      }
      require(measured.wrist == original.wrist && measured.base == original.base &&
        measured.pose.x == original.pose.x && measured.pose.y == original.pose.y &&
        measured.pose.z == original.pose.z && measured.pose.phi == original.pose.phi,
        "Planning changed raw feedback or its FK pose");
      const auto normal = planner.generateRoute(measured, {{fixture.x, fixture.y,
        fixture.z + 10.0, measured.pose.phi}});
      require(std::abs(std::remainder(normal.front().phi - measured.pose.phi, turn)) < 1e-6,
        "Normal routes inherited the group-only wrist clamp");
      require(!planner.planGroup(measured, request).success,
        "The default offline checker tolerance must remain strict");
      require(!planner.planGroup(measured, request, 0.0).success,
        "Zero tolerance accepted an out-of-range measured wrist");
      const auto beyond = state(planner, fixture, boundary + outward * 6.1 * turn / 360.0);
      require(!planner.planGroup(beyond, request, tolerance).success,
        "6.1 degree feedback exceeded the configured tolerance");
      const auto on_boundary = state(planner, fixture, boundary);
      require(planner.planGroup(on_boundary, request, 0.0).success,
        "Zero tolerance rejected a legal boundary");
    }
  }
  const auto valid = state(planner, fixture, -1.0);
  nav_director::PlanRotationGroup::Request request;
  request.targets = {target(fixture)};
  request.route_ends = {1};
  for (const double invalid : {-0.001, turn / 2.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    require(!planner.planGroup(valid, request, invalid).success,
      "Invalid feedback tolerance was accepted");
  }
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
  const auto rejected = planner.planGroup(wound, request);
  require(!rejected.success, "Lower-wound A to B must exceed the small phi budget");
  require(rejected.message.find("exceeds") != std::string::npos,
    "The phi budget failure was not explained");
  const auto accepted = planner.planGroup(zero, request);
  require(accepted.success, accepted.message);
  require(accepted.phi_travel < 1e-5, "Upper-branch A to B should preserve phi");
  require(accepted.routes.size() == 1, "Route count changed");
  require(accepted.routes[0].path.header.stamp.sec == 0 &&
    accepted.routes[0].path.header.stamp.nanosec == 0, "Offline planner used a clock");
  require(std::abs(accepted.routes[0].wrist_angles.back() + 0.244978663) < 1e-5,
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
  require(std::abs(lookahead.routes[0].wrist_angles.back()) < 1e-5,
    "Lookahead did not select the useful winding at A");

  request.limit_phi_travel = false;
  catchrobo2026_msgs::msg::PhiTravelInterval interval;
  interval.start_target = 1;
  interval.end_target = 2;
  interval.max_phi_travel = turn / 12.0;
  request.phi_travel_intervals = {interval};
  const auto scoped = planner.planGroup(pick_state, request);
  require(scoped.success, scoped.message);
  require(scoped.interval_phi_travel.size() == 1 && scoped.interval_phi_travel[0] < 1e-5,
    "Only A to B should be counted against the 30-degree budget");
  require(scoped.phi_travel > 3.0 && scoped.phi_travel < 3.2,
    "The approach must remain outside the local budget");
  require(std::abs(scoped.routes[0].wrist_angles.back()) < 1e-5,
    "The local bound must influence winding selection before its start");

  request.targets = {target({150.0, 0.0, 360.0, 0.0}),
    target({150.87, -436.0, 390.0, 0.0})};
  request.route_ends = {2};
  request.phi_travel_intervals[0].start_target = 1;
  request.phi_travel_intervals[0].end_target = 2;
  const auto waypoint_scoped = planner.planGroup(pick_state, request);
  require(waypoint_scoped.success, waypoint_scoped.message);
  require(waypoint_scoped.routes.size() == 1, "Waypoint route was split by its phi interval");
  require(waypoint_scoped.interval_phi_travel.size() == 1 &&
    waypoint_scoped.interval_phi_travel[0] <= turn / 12.0 + 1e-5,
    "Waypoint-to-place phi travel exceeded 30 degrees");
  const auto & waypoint_route = waypoint_scoped.routes.front();
  require(waypoint_route.target_sample_indices.size() == 2,
    "Waypoint route lost target sample metadata");
  require(waypoint_route.target_sample_indices[0] > 0 &&
    waypoint_route.target_sample_indices[0] < waypoint_route.target_sample_indices[1] &&
    waypoint_route.target_sample_indices[1] + 1 == waypoint_route.path.poses.size(),
    "Waypoint and terminal target samples are not exact ordered boundaries");
  for (size_t i = 0; i < waypoint_route.target_sample_indices.size(); ++i) {
    const auto & sample = waypoint_route.path.poses[waypoint_route.target_sample_indices[i]].pose;
    const auto & expected = request.targets[i];
    require(std::abs(sample.position.x - expected.position.x) < 1e-12 &&
      std::abs(sample.position.y - expected.position.y) < 1e-12 &&
      std::abs(sample.position.z - expected.position.z) < 1e-12,
      "Target sample metadata points at an interpolated pose");
  }
  for (const double wrist : waypoint_route.wrist_angles) {
    require(wrist >= -turn - 1e-8 && wrist <= 1e-8,
      "Waypoint route selected a wrapped wrist command outside its limits");
  }

  request.targets = {target(a), target(b)};
  request.route_ends = {1, 2};
  request.phi_travel_intervals[0] = interval;
  auto from_a = request;
  from_a.targets = {target(b)};
  from_a.route_ends = {1};
  from_a.phi_travel_intervals[0].start_target = 0;
  from_a.phi_travel_intervals[0].end_target = 1;
  require(!planner.planGroup(wound, from_a).success,
    "Lower-wound A to B must be rejected when the interval is active from the start");
  request.limit_phi_travel = true;
  request.max_phi_travel = 0.6;
  require(!planner.planGroup(pick_state, request).success, "Local bound must not replace global bound");
  request.limit_phi_travel = false;
  request.allow_wrist_reversal = false;
  require(!planner.planGroup(pick_state, request).success, "Direction bound must remain active");
  request.allow_wrist_reversal = true;
  for (const auto range : std::vector<std::array<uint32_t, 2>>{{2, 1}, {1, 1}, {0, 3}}) {
    request.phi_travel_intervals[0].start_target = range[0];
    request.phi_travel_intervals[0].end_target = range[1];
    require(!planner.planGroup(pick_state, request).success, "Invalid interval indices were accepted");
  }
  for (const nav_director::Point3D rear : std::vector<nav_director::Point3D>{
      {1199.13, -586.0, 304.35, -turn / 2.0},
      {1235.87, -622.0, 304.35, -turn / 2.0},
      {1199.13, -736.0, 304.35, -turn / 2.0}}) {
    std::array<float, 4> rear_joints{};
    require(planner.solveIK(rear, rear_joints), "Blue rear fixture must be reachable");
    for (const auto & ends : std::vector<std::vector<uint32_t>>{{1, 2}, {2}}) {
      auto fallback_request = request;
      fallback_request.route_ends = ends;
      fallback_request.phi_travel_intervals = {interval};
      const auto fallback = planner.planGroup(
        planner.stateFromJoints(rear_joints), fallback_request);
      require(fallback.success, fallback.message);
      require(fallback.routes.size() == ends.size() && fallback.routes[0].phi_angles.empty(),
        "Rear approach must use the bounded wrist interpolation fallback");
      if (ends.size() == 1) {
        require(fallback.routes[0].target_sample_indices.size() == 2 &&
          fallback.routes[0].target_sample_indices[0] > 0 &&
          fallback.routes[0].target_sample_indices[0] <
          fallback.routes[0].target_sample_indices[1],
          "Fallback lost its internal target boundary");
        for (size_t i = 0; i < fallback_request.targets.size(); ++i) {
          const auto & sample = fallback.routes[0].path.poses[
            fallback.routes[0].target_sample_indices[i]].pose;
          require(std::abs(sample.position.x - fallback_request.targets[i].position.x) < 1e-12 &&
            std::abs(sample.position.y - fallback_request.targets[i].position.y) < 1e-12 &&
            std::abs(sample.position.z - fallback_request.targets[i].position.z) < 1e-12,
            "Fallback target sample metadata is not exact");
        }
      }
      require(fallback.interval_phi_travel.size() == 1 &&
        fallback.interval_phi_travel[0] < turn / 12.0,
        "Fallback must include the interior phi reversal in the A-to-B budget");
      require(fallback.interval_phi_travel[0] > 0.04, "Fallback interior travel was lost");
    }
  }
  request.phi_travel_intervals = {interval, interval};
  require(!planner.planGroup(pick_state, request).success, "Overlapping intervals were accepted");

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
    checkFeedbackTolerance();
    std::cout << "Offline route planner checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
