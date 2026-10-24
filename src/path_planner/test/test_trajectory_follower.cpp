#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "path_planner/robot_model.hpp"
#include "path_planner/trajectory_follower.hpp"
#include "path_planner/trajectory_node.hpp"
#include "path_planner/trajectory_pipeline.hpp"

namespace
{
using path_planner::CartesianPose;
using path_planner::FollowRouteGoal;
using path_planner::FollowTrajectoryGoalRequest;
using path_planner::KinematicChain;
using path_planner::RouteWaypoint;
using path_planner::State;
using path_planner::TrajectoryFollower;
using path_planner::TrajectoryFollowerConfig;
using path_planner::TrajectoryNode;
using path_planner::TrajectoryPipeline;
using path_planner::TrajectoryPipelineConfig;

void testForwardAndInverseKinematics()
{
  const KinematicChain chain = path_planner::defaultKinematicChain();
  const std::vector<State> test_states = {
    {-0.7582, 0.7605, 2.1702, -0.8126},
    {0.7582, 0.7605, 2.1702, -2.3290},
    {0.0, 0.45, 1.85, -1.20},
    {0.35, -0.10, 1.40, -0.50}};

  for (const auto & q : test_states) {
    const CartesianPose pose = path_planner::forwardKinematicsPose(chain, q);
    State q_ik;
    const bool ok = path_planner::inverseKinematics(chain, pose, q_ik, true);
    assert(ok);
    (void)ok;
    const CartesianPose pose_check = path_planner::forwardKinematicsPose(chain, q_ik);
    assert(std::abs(pose.x_mm - pose_check.x_mm) < 1e-6);
    assert(std::abs(pose.y_mm - pose_check.y_mm) < 1e-6);
    assert(std::abs(pose.z_mm - pose_check.z_mm) < 1e-6);
    (void)pose_check;
    for (std::size_t i = 0; i < 4; ++i) {
      assert(std::abs(q[i] - q_ik[i]) < 1e-6);
    }
  }


  State unreachable;
  assert(!path_planner::inverseKinematics(chain, {5000.0, 5000.0, 2000.0, 0.0}, unreachable));
}

void testJointFeedbackDiagnostics()
{
  TrajectoryFollower follower;
  assert(follower.jointFeedbackDiagnostic(1.0).find("reason=not_received") != std::string::npos);

  follower.updateCurrentJoints({0.1, 0.2}, 1.0);
  assert(follower.jointFeedbackDiagnostic(1.1).find("reason=too_few_values") != std::string::npos);

  follower.updateCurrentJoints(
    {0.0, 0.5, 1.5, std::numeric_limits<double>::quiet_NaN()}, 1.2);
  assert(follower.jointFeedbackDiagnostic(1.3).find("reason=non_finite") != std::string::npos);


  follower.updateCurrentJoints({0.0, 0.5, 1.5, 0.5}, 1.4);
  assert(
    follower.jointFeedbackDiagnostic(1.5).find("reason=wrist_out_of_range") != std::string::npos);


  follower.updateCurrentJoints({0.0, 0.5, 1.5, -1.0}, 2.0);
  assert(follower.hasFreshJoints(2.5, true));
  assert(follower.jointFeedbackDiagnostic(2.5).find("reason=ok") != std::string::npos);
  assert(!follower.hasFreshJoints(3.5, true));
  assert(follower.jointFeedbackDiagnostic(3.5).find("reason=stale") != std::string::npos);
}

RouteWaypoint makeWaypointFromTheta(const KinematicChain & chain, const State & q)
{
  const CartesianPose cart = path_planner::forwardKinematicsPose(chain, q);
  RouteWaypoint wp;
  wp.x_m = cart.x_mm / 1000.0;
  wp.y_m = cart.y_mm / 1000.0;
  wp.z_m = cart.z_mm / 1000.0;
  wp.qx = 0.0;
  wp.qy = 0.0;
  wp.qz = std::sin(cart.phi_rad * 0.5);
  wp.qw = std::cos(cart.phi_rad * 0.5);
  return wp;
}

void testFollowRouteNormalAndConstrained()
{
  const KinematicChain chain = path_planner::defaultKinematicChain();
  TrajectoryFollower follower([] {TrajectoryFollowerConfig c;c.enable_simulation_torque=true;return c;}(), chain);

  const State q_start{-0.30, 0.65, 1.95, -0.80};
  const State q_goal{0.30, 0.65, 1.95, -1.40};
  double now_sec = 10.0;
  follower.updateCurrentJoints(q_start, now_sec);

  FollowRouteGoal route_goal;
  route_goal.start = true;
  const int samples = 25;
  for (int i = 0; i <= samples; ++i) {
    const double a = static_cast<double>(i) / static_cast<double>(samples);
    State q(4, 0.0);
    for (std::size_t j = 0; j < 4; ++j) {
      q[j] = (1.0 - a) * q_start[j] + a * q_goal[j];
    }
    route_goal.poses.push_back(makeWaypointFromTheta(chain, q));
  }

  std::string reason;
  assert(follower.validateAndStartRoute(route_goal, now_sec, &reason));
  assert(follower.isActive());

  path_planner::FollowerStepOutput out;
  for (int step = 0; step < 300 && follower.isActive(); ++step) {
    now_sec += 0.05;
    out = follower.stepRoute(now_sec);
    assert(!out.aborted);
    follower.stepInternalPlant(
      out.target_joint_angles, out.target_joint_velocities, out.command_torque, 0.05, now_sec);
    if (out.finished) {
      break;
    }
  }
  assert(out.finished && out.success);
  assert(out.position_error_mm <= 30.0);
  assert(out.yaw_error_rad <= 0.05);


  FollowRouteGoal constrained_goal = route_goal;
  constrained_goal.rotation_group_id = 7;
  constrained_goal.wrist_direction = -1;
  for (int i = 0; i <= samples; ++i) {
    const double a = static_cast<double>(i) / static_cast<double>(samples);
    constrained_goal.wrist_angles.push_back((1.0 - a) * q_start[3] + a * q_goal[3]);
  }


  constrained_goal.wrist_direction = 1;
  follower.updateCurrentJoints(q_start, now_sec);
  assert(!follower.validateAndStartRoute(constrained_goal, now_sec, &reason));


  constrained_goal.wrist_direction = -1;
  assert(follower.validateAndStartRoute(constrained_goal, now_sec, &reason));
  for (int step = 0; step < 300 && follower.isActive(); ++step) {
    now_sec += 0.05;
    out = follower.stepRoute(now_sec);
    assert(!out.aborted);
    follower.stepInternalPlant(
      out.target_joint_angles, out.target_joint_velocities, out.command_torque, 0.05, now_sec);
    if (out.finished) {
      break;
    }
  }
  assert(out.finished && out.success);
}

void testFollowTrajectoryAndPlanAction()
{
  TrajectoryPipelineConfig pipe_cfg;
  pipe_cfg.hardware_output_enabled = false;
  pipe_cfg.candidate_count = 4;
  pipe_cfg.joint_bounds = {{-2.8, -1.8, -1.8, -3.2}, {2.8, 1.8, 2.8, 4.2}};
  pipe_cfg.max_velocity = {1.5, 2.0, 2.0, 3.0};
  pipe_cfg.max_acceleration = {2.5, 4.0, 4.0, 6.0};
  pipe_cfg.max_jerk = {25.0, 40.0, 40.0, 60.0};

  path_planner::RobotDynamicsConfig dyn_cfg;
  dyn_cfg.source = "unit_test";
  dyn_cfg.measured_at = "2026-10-22";
  dyn_cfg.safety_factor = 0.85;
  for (std::size_t i = 0; i < path_planner::kUrdfJointCount; ++i) {
    dyn_cfg.links[i].mass = 0.6;
    dyn_cfg.links[i].center_of_mass = {0.0, 0.0, 0.1};
    dyn_cfg.links[i].inertia(0, 0) = 0.01;
    dyn_cfg.links[i].inertia(1, 1) = 0.01;
    dyn_cfg.links[i].inertia(2, 2) = 0.01;
  }
  dyn_cfg.theta1.continuous_torque = 1.5;
  dyn_cfg.theta1.peak_torque = 2.5;
  dyn_cfg.theta1.gear_ratio = 10.0;
  dyn_cfg.theta1.efficiency = 0.90;
  dyn_cfg.theta1.positive_speed_torque_curve = {{0.0, 2.5}, {2.0, 1.5}};
  dyn_cfg.theta1.negative_speed_torque_curve = {{0.0, 2.5}, {2.0, 1.5}};

  auto pipeline = std::make_shared<TrajectoryPipeline>(pipe_cfg, dyn_cfg);
  TrajectoryFollowerConfig follower_cfg;follower_cfg.enable_simulation_torque=true;
  TrajectoryNode node(pipeline,path_planner::ConcurrencyPolicy::kReject,follower_cfg);

  const State q_start{-0.40, 0.70, 2.00, -0.90};
  const State q_goal{0.40, 0.70, 2.00, -1.70};
  double now_sec = 0.0;
  node.follower()->updateCurrentJoints(q_start, now_sec);

  FollowTrajectoryGoalRequest goal;
  goal.start = true;
  goal.plan_from_start_goal = true;
  goal.start_positions = q_start;
  goal.goal_positions = q_goal;
  goal.seed = 42;

  path_planner::FollowTrajectoryActionResult fail_res;
  assert(node.acceptFollowTrajectoryGoal(goal, now_sec, &fail_res));
  assert(node.follower()->isActive());

  path_planner::FollowerStepOutput step_out;
  for (int iter = 0; iter < 600 && node.follower()->isActive(); ++iter) {
    now_sec += 0.02;
    step_out = node.follower()->stepTimedTrajectory(now_sec);
    assert(!step_out.aborted);
    node.follower()->stepInternalPlant(
      step_out.target_joint_angles,
      step_out.target_joint_velocities,
      step_out.command_torque,
      0.02,
      now_sec);
    if (step_out.finished) {
      break;
    }
  }
  assert(step_out.finished && step_out.success);
  assert(step_out.position_error_mm <= 30.0);
  assert(step_out.yaw_error_rad <= 0.05);
  node.follower()->updateCurrentJoints(q_start,now_sec);
  FollowTrajectoryGoalRequest stored;stored.start=true;
  pipeline->updateCollisionScene({});
  assert(!node.acceptFollowTrajectoryGoal(stored,now_sec,&fail_res));
  assert(node.acceptFollowTrajectoryGoal(goal,now_sec,&fail_res));
  pipeline->updateCollisionScene({});
  assert(node.stepFollowTrajectory(now_sec+.01).aborted);
}
void testFailureGuardsAndPhysicalAxes()
{
  const State q{0.0,0.5,1.5,-1.0};
  path_planner::DynamicState a,b;a.position=q;a.time=0;b=a;b.time=.1;
  path_planner::TimedTrajectory trajectory;trajectory.points={a,b};
  TrajectoryFollower absent;
  assert(!absent.startTimedTrajectory(trajectory,0));
  FollowRouteGoal route_goal;route_goal.poses={makeWaypointFromTheta(path_planner::defaultKinematicChain(),q)};
  assert(!absent.validateAndStartRoute(route_goal,0));
  TrajectoryFollower timed;timed.updateCurrentJoints(q,0);
  assert(timed.startTimedTrajectory(trajectory,0));assert(!timed.stepTimedTrajectory(.1).finished);
  timed.updateCurrentJoints(q,.2);auto stale=timed.stepTimedTrajectory(100);
  assert(stale.aborted && !stale.success && stale.target_joint_angles.empty());
  TrajectoryFollower route;route.updateCurrentJoints(q,0);assert(route.validateAndStartRoute(route_goal,0));
  route.stepRoute(.1);route.updateCurrentJoints(q,.2);assert(route.stepRoute(100).aborted);
  TrajectoryFollower malformed;malformed.updateCurrentJoints(q,0);
  auto bad=trajectory;bad.points[0].time=-.5;assert(!malformed.startTimedTrajectory(bad,0));
  bad=trajectory;bad.points[1].time=0;assert(!malformed.startTimedTrajectory(bad,0));
  bad=trajectory;bad.points[1].velocity={0,0,0};assert(!malformed.startTimedTrajectory(bad,0));
  bad=trajectory;bad.points[1].acceleration={0,0,std::numeric_limits<double>::quiet_NaN(),0};
  assert(!malformed.startTimedTrajectory(bad,0));
  bad=trajectory;bad.points[0].position[0]=.5;assert(!malformed.startTimedTrajectory(bad,0));
  malformed.updateCurrentJoints(q,100);assert(!malformed.hasFreshJoints(0));
  TrajectoryFollower lag;lag.updateCurrentJoints(q,0);
  auto moving=trajectory;moving.points[1].time=1;moving.points[1].position[0]=1;
  moving.points[0].velocity=moving.points[1].velocity=State{1,0,0,0};
  assert(lag.startTimedTrajectory(moving,0));lag.stepTimedTrajectory(.05);
  auto paused=lag.stepTimedTrajectory(.1);assert(paused.target_joint_velocities[0]==0 && paused.command_torque.empty());
  TrajectoryFollowerConfig cfg;cfg.active_axes=3;cfg.execution_timeout_sec=.5;
  TrajectoryFollower three(cfg);State q3{0,.5,1.5,0};three.updateCurrentJoints({0,.5,1.5},0);assert(!three.hasValidJoints());
  three.updateCurrentJoints(q,0);assert(!three.hasValidJoints());
  three.updateCurrentJoints(q3,0);assert(three.hasValidJoints());
  FollowRouteGoal xyz;xyz.poses={makeWaypointFromTheta(path_planner::defaultKinematicChain(),q3)};
  xyz.poses[0].qz=std::sin(2.0/2);xyz.poses[0].qw=std::cos(2.0/2);
  assert(three.validateAndStartRoute(xyz,0));auto first=three.stepRoute(.01);
  assert(!first.finished && first.target_joint_angles[3]==0 && first.command_torque.empty());
  three.updateCurrentJoints(q3,.02);auto reached=three.stepRoute(.02);assert(reached.success);
  auto fixed=trajectory;fixed.points[0].position=q3;fixed.points[1].position=q3;
  assert(three.startTimedTrajectory(fixed,.02));three.updateCurrentJoints(q3,.6);
  assert(three.stepTimedTrajectory(.6).aborted);
  three.updateCurrentJoints(q3,.6);assert(three.startTimedTrajectory(fixed,.6));three.cancel();
  assert(!three.isActive() && three.stepTimedTrajectory(.61).target_joint_angles.empty());
  fixed.points[1].position[3]=.01;assert(!three.startTimedTrajectory(fixed,.61));
}

}  // namespace

int main()
{
  testFailureGuardsAndPhysicalAxes();
  testForwardAndInverseKinematics();
  testJointFeedbackDiagnostics();
  testFollowRouteNormalAndConstrained();
  testFollowTrajectoryAndPlanAction();
  std::cout << "[test_trajectory_follower] ALL TESTS PASSED" << std::endl;
  return 0;
}
