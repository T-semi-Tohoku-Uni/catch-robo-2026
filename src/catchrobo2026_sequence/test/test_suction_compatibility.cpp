#include "catchrobo2026_sequence/sequence_config.hpp"

#include <gtest/gtest.h>

#include <string>

namespace catchrobo2026_sequence
{
namespace
{

SequenceConfig configuration(const std::string & steps)
{
  return SequenceConfig::from_yaml(
    "version: 1\nposes: {}\nsequences:\n  s:\n    steps:\n" + steps +
    "\nbindings: {red: {pick: {'0,1': s}, place: {}}, blue: {pick: {}, place: {}}}\n");
}

const std::string check =
  "      - suction_check: {start: {timeout: 0.1}}\n"
  "      - suction_check: wait\n";

TEST(SuctionCompatibility, InlineWaypointsKeepAnchorsCorrelatedWithBranches)
{
  const auto config = configuration(check + R"(
      - if:
          condition: suction_success
          then: [{move: {absolute: [100, 200, 300, 0]}}]
          else: [{move: {absolute: [400, 500, 600, 0]}}]
      - if:
          condition: suction_success
          then:
            - move: {absolute: [10, 20, 30, 0], waypoints: [{relative: [1, 2, 3, 0]}]}
          else:
            - move: {absolute: [40, 50, 60, 0], waypoints: [{relative: [4, 5, 6, 0]}]}
)");
  std::vector<Pose> points;
  for (const auto & step : config.compile("red", "pick", 0, 1)) {
    points.insert(points.end(), step.waypoints.begin(), step.waypoints.end());
  }
  EXPECT_EQ(points, (std::vector<Pose>{{101, 202, 303, 0}, {404, 505, 606, 0}}));
}

TEST(SuctionCompatibility, InlineWaypointsRejectAmbiguousBranchAnchors)
{
  EXPECT_THROW(configuration(check + R"(
      - if:
          condition: suction_success
          then: [{move: {absolute: [100, 200, 300, 0]}}]
          else: [{move: {absolute: [400, 500, 600, 0]}}]
      - move: {absolute: [10, 20, 30, 0], waypoints: [{relative: [1, 2, 3, 0]}]}
)"), ConfigError);
}

TEST(SuctionCompatibility, CompleteGroupsCanBeSelectedByACondition)
{
  const auto config = configuration(check + R"(
      - if:
          condition: suction_success
          then:
            - sequence_group: start
            - phi_travel: {start: true, limit: 0.5}
            - waypoint: {absolute: [100, 200, 300, 0]}
            - move: {absolute: [120, 200, 300, 0]}
            - phi_travel: end
            - sequence_group: end
          else: [{pump: off}]
)");
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 11u);
  EXPECT_EQ(steps[2].jump_index, 10u);
  EXPECT_EQ(steps[9].jump_index, steps.size());
  const auto intervals = collect_phi_travel_intervals(steps, 3, 8);
  ASSERT_EQ(intervals.size(), 1u);
  EXPECT_EQ(intervals[0].end_target, 2u);
}

TEST(SuctionCompatibility, GroupCannotContainConditionsOrEscapingBreaks)
{
  EXPECT_THROW(configuration(check + R"(
      - sequence_group: start
      - if: {condition: suction_success, then: [{pump: off}]}
      - move: {absolute: [100, 200, 300, 0]}
      - sequence_group: end
)"), ConfigError);
  EXPECT_THROW(configuration(R"(
      - for:
          max_iterations: 2
          steps:
            - sequence_group: start
            - move: {absolute: [100, 200, 300, 0]}
            - break: true
            - sequence_group: end
)"), ConfigError);
}

TEST(SuctionCompatibility, BranchCannotJumpIntoAnActiveGroup)
{
  EXPECT_THROW(configuration(check + R"(
      - if:
          condition: suction_success
          then: [{sequence_group: start}]
      - move: {absolute: [100, 200, 300, 0]}
      - sequence_group: end
)"), ConfigError);
}

TEST(SuctionCompatibility, DeferredWaypointsMustBeCommonAfterTheBranch)
{
  EXPECT_THROW(configuration(check + R"(
      - if:
          condition: suction_success
          then: [{waypoint: {absolute: [100, 200, 300, 0]}}]
)"), ConfigError);
}

TEST(SuctionCompatibility, BreakToDeferredTailRemainsAValidCompletionIndex)
{
  const auto config = configuration(R"(
      - move: {absolute: [100, 200, 300, 0]}
      - for:
          max_iterations: 2
          steps:
            - suction_check: {start: {timeout: 0.1}}
            - suction_check: wait
            - if: {condition: suction_success, then: [{break: true}]}
      - waypoint: {relative: [1, 2, 3, 0]}
)");
  const auto prepared = prepare_sequence(config.compile("red", "pick", 0, 1));
  EXPECT_EQ(prepared.deferred_waypoints, (std::vector<Pose>{{101, 202, 303, 0}}));
  for (const auto & step : prepared.steps) {
    if (step.type == StepType::JUMP) {
      EXPECT_EQ(step.jump_index, prepared.steps.size());
    }
  }
}

TEST(SuctionCompatibility, PendingWaypointsRejectAConditionalFirstMove)
{
  const auto config = configuration(check + R"(
      - if:
          condition: suction_success
          then: [{move: {absolute: [100, 200, 300, 0]}}]
      - move: {absolute: [400, 500, 600, 0]}
)");
  const auto steps = config.compile("red", "pick", 0, 1);
  EXPECT_NO_THROW(prepare_sequence(steps));
  EXPECT_THROW(prepare_sequence(steps, {{10, 20, 30, 0}}), ConfigError);
}

TEST(SuctionCompatibility, PendingWaypointsBeforeRetryLeaveJumpsUnchanged)
{
  const auto config = configuration("      - move: {absolute: [100, 200, 300, 0]}\n" +
    check + "      - if: {condition: suction_success, then: [{pump: suction}]}\n");
  const auto steps = config.compile("red", "pick", 0, 1);
  const auto prepared = prepare_sequence(steps, {{10, 20, 30, 0}});
  EXPECT_EQ(prepared.consumed_pending_waypoints, 1u);
  EXPECT_EQ(prepared.steps[0].waypoints, (std::vector<Pose>{{10, 20, 30, 0}}));
  EXPECT_EQ(prepared.steps[3].jump_index, steps[3].jump_index);
}

TEST(SuctionCompatibility, UnrolledLoopBudgetIncludesInlineWaypoints)
{
  std::string points;
  for (int i = 0; i < 100; ++i) {
    points += (i ? ", " : "") + std::string("[1, 2, 3, 0]");
  }
  EXPECT_THROW(configuration(
      "      - for:\n          max_iterations: 100\n          steps:\n"
      "            - move: {absolute: [100, 200, 300, 0], waypoints: [" + points + "]}\n"),
    ConfigError);
}

}  // namespace
}  // namespace catchrobo2026_sequence
