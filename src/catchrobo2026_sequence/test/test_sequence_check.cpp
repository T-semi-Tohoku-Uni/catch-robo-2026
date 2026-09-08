#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "catchrobo2026_sequence/sequence_check.hpp"
#include "nav_director/route_planner.hpp"

namespace
{
using catchrobo2026_sequence::CheckJoints;
using catchrobo2026_sequence::CheckStatus;
using catchrobo2026_sequence::Pose;
using catchrobo2026_sequence::SequenceCheckOptions;
using catchrobo2026_sequence::SequenceCheckResult;
using catchrobo2026_sequence::Step;
using catchrobo2026_sequence::StepType;
using catchrobo2026_sequence::check_sequence_steps;

constexpr double kPi = 3.14159265358979323846;
const Pose kEndingA{675, 200, 300, 0};
const Pose kEndingB{670, -110, 220, 0};
const Pose kPickRetreat{375, 238, 196.95, -kPi};

CheckJoints joints_at(const Pose & pose)
{
  nav_director::RoutePlanner planner;
  CheckJoints joints{};
  if (!planner.solveIK({pose[0], pose[1], pose[2], pose[3]}, joints)) {
    throw std::runtime_error("Test start pose is unreachable");
  }
  return joints;
}

Step move_to(const Pose & pose)
{
  Step step;
  step.pose = pose;
  return step;
}

Step flag(StepType type)
{
  Step step;
  step.type = type;
  return step;
}

Step group(bool one_direction, std::optional<double> max_phi = std::nullopt)
{
  auto step = flag(StepType::SEQUENCE_START);
  step.rotation_group = one_direction;
  step.max_phi_travel = max_phi;
  return step;
}

bool has_code(const SequenceCheckResult & result, const std::string & code)
{
  for (const auto & diagnostic : result.diagnostics) {
    if (diagnostic.code == code) {
      return true;
    }
  }
  return false;
}

TEST(SequenceCheck, NormalEndingReportsFullTurnWithoutInventingLimit)
{
  const auto result = check_sequence_steps({move_to(kEndingB)}, joints_at(kEndingA));
  EXPECT_EQ(result.status, CheckStatus::FEASIBLE) << result.message;
  EXPECT_TRUE(has_code(result, "wrist_wrap_jump"));
  EXPECT_GT(result.max_wrist_step, 6.0);
  EXPECT_GT(result.phi_travel, 6.0);
  EXPECT_EQ(result.route_count, 1u);
  EXPECT_GT(result.sample_count, 2u);
  ASSERT_TRUE(result.final_joints);
  EXPECT_NEAR((*result.final_joints)[3], -6.2207665, 1e-4);
}

TEST(SequenceCheck, CurrentOneDirectionEndingFailsBeforeFirstRoute)
{
  const auto result = check_sequence_steps({group(true), move_to(kEndingA),
      move_to(kEndingB), flag(StepType::SEQUENCE_END)}, joints_at(kPickRetreat));
  EXPECT_EQ(result.status, CheckStatus::INFEASIBLE);
  EXPECT_TRUE(has_code(result, "sequence_group_infeasible"));
  EXPECT_EQ(result.route_count, 0u);
  EXPECT_EQ(result.sample_count, 0u);
  EXPECT_FALSE(result.final_joints);
  ASSERT_FALSE(result.diagnostics.empty());
  EXPECT_EQ(result.diagnostics.front().step_index, 0u);
}

TEST(SequenceCheck, ShortPhiEndingRequiresCorrectStartingWinding)
{
  const std::vector<Step> steps{group(false, 10.0 * kPi / 180.0),
    move_to(kEndingB), flag(StepType::SEQUENCE_END)};
  const auto rejected = check_sequence_steps(steps, joints_at(kEndingA));
  EXPECT_EQ(rejected.status, CheckStatus::INFEASIBLE);
  auto start = joints_at(kEndingA);
  start[3] = static_cast<float>(-2.0 * kPi);
  const auto accepted = check_sequence_steps(steps, start);
  EXPECT_EQ(accepted.status, CheckStatus::FEASIBLE) << accepted.message;
  EXPECT_LT(accepted.phi_travel, 1e-4);
  EXPECT_LT(accepted.max_wrist_step, 0.1);
  EXPECT_FALSE(has_code(accepted, "wrist_wrap_jump"));
  ASSERT_TRUE(accepted.final_joints);
  EXPECT_NEAR((*accepted.final_joints)[3], -6.2207665, 1e-4);
}

TEST(SequenceCheck, PhiPlannerLooksAcrossMovesAndMechanismSteps)
{
  const auto result = check_sequence_steps({group(false, kPi + 0.01), move_to(kEndingA),
      flag(StepType::PUMP), flag(StepType::WAIT), move_to(kEndingB),
      flag(StepType::SEQUENCE_END)}, joints_at(kPickRetreat));
  EXPECT_EQ(result.status, CheckStatus::FEASIBLE) << result.message;
  EXPECT_EQ(result.route_count, 2u);
  EXPECT_NEAR(result.phi_travel, kPi, 1e-3);
  EXPECT_FALSE(has_code(result, "wrist_wrap_jump"));
}

TEST(SequenceCheck, ConsecutiveWaypointsAndInlinePointsFormOneRoute)
{
  auto waypoint = move_to({580, 180, 290, -kPi});
  waypoint.waypoint = true;
  waypoint.waypoints.push_back({550, 180, 290, -kPi});
  auto end = move_to({630, 180, 290, -kPi});
  end.waypoints.push_back({610, 180, 290, -kPi});
  const auto result = check_sequence_steps({waypoint, end},
    joints_at({500, 180, 290, -kPi}));
  EXPECT_EQ(result.status, CheckStatus::FEASIBLE);
  EXPECT_EQ(result.route_count, 1u);
  ASSERT_TRUE(result.final_joints);
  nav_director::RoutePlanner planner;
  const auto state = planner.stateFromJoints(*result.final_joints);
  EXPECT_NEAR(state.pose.x, 630.0, 0.01);
  EXPECT_NEAR(state.pose.y, 180.0, 0.01);
  EXPECT_NEAR(state.pose.z, 290.0, 0.01);
}

TEST(SequenceCheck, UnreachableSampleStopsSubsequentPrediction)
{
  const auto result = check_sequence_steps({move_to({5000, 200, 300, 0}),
      move_to(kEndingA)}, joints_at(kEndingA));
  EXPECT_EQ(result.status, CheckStatus::INFEASIBLE);
  EXPECT_TRUE(has_code(result, "unreachable_sample"));
  EXPECT_EQ(result.route_count, 1u);
  EXPECT_FALSE(result.final_joints);
}

TEST(SequenceCheck, ReachableTargetsCanStillHaveUnreachableSplineInterior)
{
  const Pose start{675, 650, 90, -kPi};
  const Pose waypoint{675, 760, 90, -kPi};
  ASSERT_NO_THROW(joints_at(waypoint));
  auto move = move_to(start);
  move.waypoints = {waypoint, waypoint};
  const auto result = check_sequence_steps({move}, joints_at(start));
  EXPECT_EQ(result.status, CheckStatus::INFEASIBLE);
  EXPECT_TRUE(has_code(result, "unreachable_sample"));
  EXPECT_GT(result.sample_count, 1u);
  EXPECT_FALSE(result.final_joints);
}

TEST(SequenceCheck, MissingInitialStateDoesNotPassMotion)
{
  const auto result = check_sequence_steps({move_to(kEndingA), move_to(kEndingB)}, std::nullopt);
  EXPECT_EQ(result.status, CheckStatus::UNKNOWN);
  EXPECT_TRUE(has_code(result, "initial_state_unknown"));
  EXPECT_EQ(result.route_count, 0u);
  EXPECT_EQ(result.sample_count, 0u);
  EXPECT_FALSE(result.final_joints);
}

TEST(SequenceCheck, InitializationInvalidatesPreviousJointsUnlessProvided)
{
  const std::vector<Step> steps{flag(StepType::INITIALIZE), move_to(kEndingB)};
  const auto unknown = check_sequence_steps(steps, joints_at(kPickRetreat));
  EXPECT_EQ(unknown.status, CheckStatus::UNKNOWN);
  EXPECT_TRUE(has_code(unknown, "initialization_state_unknown"));
  EXPECT_EQ(unknown.route_count, 0u);
  EXPECT_FALSE(unknown.final_joints);

  SequenceCheckOptions options;
  options.after_initialization_joints = joints_at(kEndingA);
  const auto known = check_sequence_steps(steps, joints_at(kPickRetreat), options);
  EXPECT_EQ(known.status, CheckStatus::FEASIBLE);
  EXPECT_EQ(known.route_count, 1u);
  EXPECT_TRUE(known.final_joints);
}

TEST(SequenceCheck, MechanismsWithoutMotionNeedNoJointState)
{
  const auto result = check_sequence_steps({flag(StepType::PUMP), flag(StepType::ENDEFFECTOR),
      flag(StepType::WAIT)}, std::nullopt);
  EXPECT_EQ(result.status, CheckStatus::FEASIBLE);
  EXPECT_EQ(result.route_count, 0u);
}

TEST(SequenceCheck, InvalidInitialJointsAreRejected)
{
  auto invalid = joints_at(kEndingA);
  invalid[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(check_sequence_steps({}, invalid).status, CheckStatus::INFEASIBLE);
  invalid = joints_at(kEndingA);
  invalid[3] = 0.1f;
  EXPECT_EQ(check_sequence_steps({}, invalid).status, CheckStatus::INFEASIBLE);
  invalid[3] = -7.0f;
  EXPECT_EQ(check_sequence_steps({}, invalid).status, CheckStatus::INFEASIBLE);
}

TEST(SequenceCheck, InvalidGroupBoundaryDominatesUnknownState)
{
  const auto result = check_sequence_steps({move_to(kEndingA),
      flag(StepType::SEQUENCE_END)}, std::nullopt);
  EXPECT_EQ(result.status, CheckStatus::INFEASIBLE);
  EXPECT_TRUE(has_code(result, "planning_error"));
  EXPECT_FALSE(result.final_joints);
}

TEST(SequenceCheck, TrailingWaypointIsDeferredWithoutMovingToIt)
{
  auto waypoint = move_to({550, 180, 290, -kPi});
  waypoint.waypoint = true;
  waypoint.waypoints = {{530, 180, 290, -kPi}};
  const auto initial = joints_at({500, 180, 290, -kPi});
  const auto deferred = check_sequence_steps({waypoint}, initial);
  EXPECT_EQ(deferred.status, CheckStatus::UNKNOWN);
  EXPECT_TRUE(has_code(deferred, "pending_waypoints"));
  EXPECT_EQ(deferred.final_joints, initial);
  EXPECT_EQ(deferred.route_count, 0u);
  ASSERT_EQ(deferred.pending_waypoints.size(), 2u);
  EXPECT_EQ(deferred.pending_waypoints.front(), waypoint.waypoints.front());
  EXPECT_EQ(deferred.pending_waypoints.back(), waypoint.pose);

  SequenceCheckOptions options;
  options.pending_waypoints = deferred.pending_waypoints;
  const auto next = check_sequence_steps({move_to({600, 180, 290, -kPi})},
    deferred.final_joints, options);
  EXPECT_EQ(next.status, CheckStatus::FEASIBLE);
  EXPECT_EQ(next.route_count, 1u);
  EXPECT_EQ(next.consumed_pending_waypoints, 2u);
  EXPECT_TRUE(next.pending_waypoints.empty());
}

TEST(SequenceCheck, PendingWaypointIsCheckedOnTheFollowingRoute)
{
  auto waypoint = move_to({5000, 180, 290, -kPi});
  waypoint.waypoint = true;
  const auto initial = joints_at({500, 180, 290, -kPi});
  const auto deferred = check_sequence_steps({waypoint}, initial);
  EXPECT_EQ(deferred.status, CheckStatus::UNKNOWN);
  SequenceCheckOptions options;
  options.pending_waypoints = deferred.pending_waypoints;
  const auto next = check_sequence_steps({move_to({600, 180, 290, -kPi})}, initial, options);
  EXPECT_EQ(next.status, CheckStatus::INFEASIBLE);
  EXPECT_TRUE(has_code(next, "unreachable_sample"));
  EXPECT_EQ(next.consumed_pending_waypoints, 0u);
  EXPECT_FALSE(next.final_joints);
}

TEST(SequenceCheck, ConsumedIncomingAndNewDeferredWaypointsStayDistinct)
{
  auto tail = move_to({650, 180, 290, -kPi});
  tail.waypoint = true;
  SequenceCheckOptions options;
  options.pending_waypoints = {{550, 180, 290, -kPi}};
  const auto next = check_sequence_steps({move_to({600, 180, 290, -kPi}), tail},
    joints_at({500, 180, 290, -kPi}), options);
  EXPECT_EQ(next.status, CheckStatus::UNKNOWN);
  EXPECT_EQ(next.consumed_pending_waypoints, 1u);
  ASSERT_EQ(next.pending_waypoints.size(), 1u);
  EXPECT_EQ(next.pending_waypoints[0], tail.pose);
  ASSERT_TRUE(next.final_joints);
  nav_director::RoutePlanner planner;
  EXPECT_NEAR(planner.stateFromJoints(*next.final_joints).pose.x, 600, 0.01);
}

TEST(SequenceCheck, MechanismOnlyActionPreservesIncomingWaypoints)
{
  SequenceCheckOptions options;
  options.pending_waypoints = {{550, 180, 290, -kPi}};
  const auto initial = joints_at({500, 180, 290, -kPi});
  const auto next = check_sequence_steps({flag(StepType::PUMP)}, initial, options);
  EXPECT_EQ(next.status, CheckStatus::UNKNOWN);
  EXPECT_EQ(next.final_joints, initial);
  EXPECT_EQ(next.consumed_pending_waypoints, 0u);
  EXPECT_EQ(next.pending_waypoints, options.pending_waypoints);
}

TEST(SequenceCheck, IncomingWaypointsAreIncludedInsideTheFirstGroup)
{
  SequenceCheckOptions options;
  options.pending_waypoints = {{5000, 180, 290, -kPi}};
  const auto next = check_sequence_steps({group(false, 0.01),
      move_to({600, 180, 290, -kPi}), flag(StepType::SEQUENCE_END)},
    joints_at({500, 180, 290, -kPi}), options);
  EXPECT_EQ(next.status, CheckStatus::INFEASIBLE);
  EXPECT_TRUE(has_code(next, "sequence_group_infeasible"));
  EXPECT_EQ(next.route_count, 0u);
}

TEST(SequenceCheck, ExplicitBoundaryDiscardsPendingBeforeItsMotion)
{
  SequenceCheckOptions options;
  options.pending_waypoints = {{5000, 180, 290, -kPi}};
  options.clear_pending_waypoints = true;
  const auto next = check_sequence_steps({move_to({600, 180, 290, -kPi})},
    joints_at({500, 180, 290, -kPi}), options);
  EXPECT_EQ(next.status, CheckStatus::FEASIBLE);
  EXPECT_EQ(next.discarded_pending_waypoints, 1u);
  EXPECT_EQ(next.consumed_pending_waypoints, 0u);
  EXPECT_TRUE(next.pending_waypoints.empty());
}

TEST(SequenceCheck, InitializationStepAlsoDiscardsIncomingWaypoints)
{
  SequenceCheckOptions options;
  options.pending_waypoints = {{5000, 180, 290, -kPi}};
  options.after_initialization_joints = joints_at({500, 180, 290, -kPi});
  const auto next = check_sequence_steps({move_to({525, 180, 290, -kPi}),
      flag(StepType::INITIALIZE),
      move_to({600, 180, 290, -kPi})}, options.after_initialization_joints, options);
  EXPECT_EQ(next.status, CheckStatus::FEASIBLE);
  EXPECT_EQ(next.discarded_pending_waypoints, 1u);
  EXPECT_EQ(next.consumed_pending_waypoints, 0u);
}

}  // namespace
