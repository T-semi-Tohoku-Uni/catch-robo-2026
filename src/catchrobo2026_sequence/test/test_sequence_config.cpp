#include "catchrobo2026_sequence/sequence_config.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

namespace catchrobo2026_sequence
{
namespace
{

std::string document(
  const std::string & poses, const std::string & sequences,
  const std::string & red_pick = "{}", const std::string & red_place = "{}",
  const std::string & blue_pick = "{}", const std::string & blue_place = "{}")
{
  return "version: 1\nposes: " + poses + "\nsequences: " + sequences +
         "\nbindings:\n  red:\n    pick: " + red_pick + "\n    place: " + red_place +
         "\n  blue:\n    pick: " + blue_pick + "\n    place: " + blue_place + "\n";
}

std::string movement_with_waypoints(std::size_t count)
{
  std::string result = "{move: {absolute: [1, 2, 3, 0], waypoints: [";
  for (std::size_t i = 0; i < count; ++i) {
    result += (i == 0 ? "" : ", ");
    result += "[1, 2, 3, 0]";
  }
  return result + "]}}";
}

class TemporaryConfig
{
public:
  TemporaryConfig()
  {
    char name[] = "/tmp/catchrobo-sequence-config-XXXXXX";
    const int fd = mkstemp(name);
    if (fd < 0) {
      throw std::runtime_error("mkstemp failed");
    }
    close(fd);
    path = name;
  }

  ~TemporaryConfig() {std::remove(path.c_str());}

  void write(const std::string & text) const
  {
    std::ofstream output(path);
    output << text;
    if (!output) {
      throw std::runtime_error("temporary config write failed");
    }
  }

  std::string path;
};

TEST(SequenceConfig, OptionalRouteTimeoutResolvesValuesAndValidBounds)
{
  const auto yaml = document("{}", "{}");
  EXPECT_FALSE(SequenceConfig::from_yaml(yaml).route_timeout_sec().has_value());
  const auto config = SequenceConfig::from_yaml(yaml +
      "route_timeout_sec: '$route_limit'\nvalues: {route_limit: '$limit', limit: 60}\n");
  ASSERT_TRUE(config.route_timeout_sec().has_value());
  EXPECT_DOUBLE_EQ(config.route_timeout_sec().value(), 60.0);
  for (const double value : {0.01, 86400.0}) {
    EXPECT_DOUBLE_EQ(SequenceConfig::from_yaml(yaml +
        "route_timeout_sec: " + std::to_string(value) + "\n").route_timeout_sec().value(), value);
  }
}

TEST(SequenceConfig, RejectsInvalidRouteTimeoutBeforeExecution)
{
  const auto yaml = document("{}", "{}");
  for (const std::string value : {
      "0", "-1", "86400.1", ".nan", ".inf", "null", "true", "[]", "{}", "'$missing'"})
  {
    SCOPED_TRACE(value);
    EXPECT_THROW(SequenceConfig::from_yaml(yaml + "route_timeout_sec: " + value + "\n"),
      ConfigError);
  }
  EXPECT_THROW(SequenceConfig::from_yaml(yaml +
      "route_timeout_sec: 30\nroute_timeout_sec: 60\n"), ConfigError);
  EXPECT_THROW(SequenceConfig::from_yaml(yaml +
      "route_timeout_sec: '$limit'\nvalues: {limit: 0}\n"), ConfigError);
}

TEST(SequenceConfig, ResolvesAliasesInheritanceAndFixedRelativeAnchor)
{
  const auto config = SequenceConfig::from_yaml(document(R"(
  above: [100, 200, 300, 0.5]
  raised: {extends: above, z: 400}
  target: raised
)", R"(
  approach:
    steps:
      - move: {absolute: target}
  grip:
    steps:
      - move: {relative: [10, 20, -50, 0.2]}
      - pump: suction
      - wait: 0.25
      - endeffector: 1
      - move: {relative: [0, 0, 0, 0]}
  grip_alias: grip
  target_pick:
    extends: approach
    steps:
      - call: grip_alias
  target_alias: target_pick
)", "{'0,1': target_alias}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 6u);
  EXPECT_EQ(steps[0].type, StepType::MOVE);
  EXPECT_EQ(steps[0].pose, (Pose{100, 200, 400, 0.5}));
  EXPECT_EQ(steps[1].pose, (Pose{110, 220, 350, 0.7}));
  EXPECT_EQ(steps[2].type, StepType::PUMP);
  EXPECT_EQ(steps[2].command, 1);
  EXPECT_EQ(steps[3].type, StepType::WAIT);
  EXPECT_DOUBLE_EQ(steps[3].seconds, 0.25);
  EXPECT_EQ(steps[4].type, StepType::ENDEFFECTOR);
  EXPECT_EQ(steps[4].command, 1);
  EXPECT_EQ(steps[5].pose, steps[0].pose);
}

TEST(SequenceConfig, StartSequenceReusesReferencesAndFixedAbsoluteAnchors)
{
  const auto config = SequenceConfig::from_yaml(document(
      "{above: [10, 20, '$height', 0]}", R"(
  approach: {steps: [{move: {absolute: above}}]}
  body:
    steps:
      - move: {relative: [0, 0, -5, 0]}
      - pump: off
      - endeffector: '$width'
      - wait: '$settle'
      - move: {relative: [0, 0, 5, 0]}
  startup:
    extends: approach
    steps:
      - call: body
  startup_alias: startup
)") + R"(
start_sequence: startup_alias
values: {height: 30, width: '$place_width', place_width: 1, settle: 0.25}
)");
  const auto steps = config.compile_start();
  ASSERT_EQ(steps.size(), 6u);
  EXPECT_EQ(steps[0].type, StepType::MOVE);
  EXPECT_EQ(steps[0].pose, (Pose{10, 20, 30, 0}));
  EXPECT_EQ(steps[1].pose, (Pose{10, 20, 25, 0}));
  EXPECT_EQ(steps[2].type, StepType::PUMP);
  EXPECT_EQ(steps[2].command, 0);
  EXPECT_EQ(steps[3].type, StepType::ENDEFFECTOR);
  EXPECT_EQ(steps[3].command, 1);
  EXPECT_EQ(steps[4].type, StepType::WAIT);
  EXPECT_DOUBLE_EQ(steps[4].seconds, 0.25);
  EXPECT_EQ(steps[5].pose, (Pose{10, 20, 35, 0}));
}

TEST(SequenceConfig, WaypointsResolveAcrossAliasesCallsAndInheritedFragments)
{
  const auto config = SequenceConfig::from_yaml(document(
      "{above: [100, 200, 300, 0.5]}", R"(
  approach: {steps: [{move: {absolute: above, waypoint: true}}]}
  approach_alias: approach
  relative:
    steps:
      - move: {relative: [10, 20, -50, 0.2], waypoint: true}
      - move: {relative: [0, 0, -100, 0], waypoint: true}
  empty: {steps: []}
  reset: {steps: [{move: {absolute: [400, 500, 600, 0], waypoint: true}}]}
  finish: {steps: [{move: {relative: [1, 2, 3, 0.1], waypoint: false}}]}
  finish_alias: finish
  combined:
    extends: [approach_alias, relative]
    steps:
      - call: empty
      - call: reset
      - call: finish_alias
      - move: {relative: [0, 0, 0, 0]}
)", "{'0,1': combined}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 6u);
  EXPECT_EQ(steps[0].pose, (Pose{100, 200, 300, 0.5}));
  EXPECT_EQ(steps[1].pose, (Pose{110, 220, 250, 0.7}));
  EXPECT_EQ(steps[2].pose, (Pose{100, 200, 200, 0.5}));
  EXPECT_EQ(steps[3].pose, (Pose{400, 500, 600, 0}));
  EXPECT_EQ(steps[4].pose, (Pose{401, 502, 603, 0.1}));
  EXPECT_EQ(steps[5].pose, steps[3].pose);
  for (std::size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].type, StepType::MOVE);
    EXPECT_EQ(steps[i].waypoint, i < 4);
  }
}

TEST(SequenceConfig, StandaloneWaypointFragmentsComposeThroughCallsAndInheritance)
{
  const auto config = SequenceConfig::from_yaml(document(
      "{pre_blue: [1200, 0, '$height', 0.5], pre_alias: pre_blue}", R"(
  place_pre_blue_waypoint:
    steps:
      - waypoint: {absolute: pre_alias}
  pre_alias: place_pre_blue_waypoint
  relative_waypoint:
    steps:
      - waypoint: {relative: [0, 10, '$offset', 0.25]}
  inherited:
    extends: pre_alias
    steps:
      - call: relative_waypoint
  place:
    steps:
      - call: inherited
      - move: {relative: [0, 0, -100, 0]}
)", "{}", "{}", "{}", "{'0,0': place}") +
    "values: {height: 400, offset: -50}\nend_sequence: place\n");
  const auto steps = config.compile("blue", "place", 0, 0);
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_EQ(steps[0].pose, (Pose{1200, 0, 400, 0.5}));
  EXPECT_EQ(steps[1].pose, (Pose{1200, 10, 350, 0.75}));
  EXPECT_EQ(steps[2].pose, (Pose{1200, 0, 300, 0.5}));
  for (std::size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].type, StepType::MOVE);
    EXPECT_EQ(steps[i].waypoint, i < 2);
    EXPECT_TRUE(steps[i].waypoints.empty());
  }
  const auto ending = config.compile_end();
  ASSERT_EQ(ending.size(), steps.size());
  EXPECT_EQ(ending.back().pose, steps.back().pose);
}

TEST(SequenceConfig, InlineWaypointsResolveAliasesValuesAndKeepFixedAbsoluteAnchor)
{
  const auto config = SequenceConfig::from_yaml(document(R"(
  via: [10, 20, '$height', 0.25]
  via_alias: via
  raised: {extends: via_alias, z: 50}
)", R"(
  s:
    steps:
      - move: {absolute: [100, 200, 300, 0.5]}
      - move:
          absolute: [500, 600, 700, 1]
          waypoints:
            - via_alias
            - ['$x', 40, 60, 0]
            - {absolute: raised}
            - {relative: [-10, 20, -50, 0.25]}
      - move:
          relative: [0, 0, -20, 0]
          waypoints:
            - {absolute: [800, 900, 1000, 2]}
            - {relative: [1, 2, 3, 0.25]}
      - move: {relative: [0, 0, 0, 0]}
)", "{'0,1': s}") + "values: {height: 30, x: '$via_x', via_x: 20}\n");
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 4u);
  EXPECT_FALSE(steps[1].waypoint);
  EXPECT_EQ(steps[1].pose, (Pose{500, 600, 700, 1}));
  EXPECT_EQ(steps[1].waypoints, (std::vector<Pose>{
      {10, 20, 30, 0.25}, {20, 40, 60, 0}, {10, 20, 50, 0.25}, {90, 220, 250, 0.75}}));
  EXPECT_EQ(steps[2].pose, (Pose{500, 600, 680, 1}));
  EXPECT_EQ(steps[2].waypoints, (std::vector<Pose>{
      {800, 900, 1000, 2}, {501, 602, 703, 1.25}}));
  EXPECT_EQ(steps[3].pose, (Pose{500, 600, 700, 1}));
  EXPECT_TRUE(steps[0].waypoints.empty());
  EXPECT_TRUE(steps[3].waypoints.empty());
}

TEST(SequenceConfig, InlineWaypointsSupportEmptyListsAndLegacyWaypointMovements)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - move:
          absolute: [100, 200, 300, 0]
          waypoint: true
          waypoints: [[10, 20, 30, 0], {absolute: [40, 50, 60, 0]}]
      - move: {relative: [0, 0, 10, 0], waypoints: []}
      - move: {absolute: [400, 500, 600, 0], waypoint: false, waypoints: []}
)", "{'0,1': s}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_TRUE(steps[0].waypoint);
  EXPECT_EQ(steps[0].waypoints, (std::vector<Pose>{{10, 20, 30, 0}, {40, 50, 60, 0}}));
  EXPECT_EQ(steps[1].pose, (Pose{100, 200, 310, 0}));
  EXPECT_FALSE(steps[1].waypoint);
  EXPECT_TRUE(steps[1].waypoints.empty());
  EXPECT_FALSE(steps[2].waypoint);
  EXPECT_TRUE(steps[2].waypoints.empty());
}

TEST(SequenceConfig, RejectsMalformedStandaloneWaypoints)
{
  for (const std::string waypoint : {
      "p", "[1, 2, 3, 0]", "null", "{}", "{absolute: missing}",
      "{absolute: [1, 2, 3, .nan]}", "{relative: [0, 0, 1]}",
      "{absolute: p, relative: [0, 0, 0, 0]}", "{absolute: p, waypoint: true}",
      "{absolute: p, waypoints: []}", "{absolute: p, speed: 1}",
      "{absolute: p, absolute: p}"})
  {
    SCOPED_TRACE(waypoint);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{p: [1, 2, 3, 0]}",
        "{s: {steps: [{waypoint: " + waypoint + "}]}}")), ConfigError);
  }
}

TEST(SequenceConfig, RejectsInvalidInlineWaypointsEvenInUnusedFragments)
{
  for (const std::string waypoints : {
      "null", "p", "{}", "[1, 2, 3, 0]", "[missing]", "[[1, 2, 3]]", "[null]",
      "[[1, 2, 3, .inf]]", "[[1, 2, '$missing', 0]]", "[{}]",
      "[{relative: [0, 0, .nan, 0]}]", "[{absolute: missing}]",
      "[{absolute: p, relative: [0, 0, 0, 0]}]",
      "[{absolute: p, waypoint: true}]", "[{absolute: p, waypoints: []}]",
      "[{absolute: p, speed: 1}]", "[{relative: p}]",
      "[{absolute: p, absolute: p}]"})
  {
    SCOPED_TRACE(waypoints);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{p: [1, 2, 3, 0]}",
        "{s: {steps: [{move: {absolute: p, waypoints: " + waypoints + "}}]}}")),
      ConfigError);
  }
  EXPECT_THROW(SequenceConfig::from_yaml(document("{p: [1, 2, 3, 0]}",
      "{s: {steps: [{move: {absolute: p, waypoints: [], waypoints: []}}]}}")),
    ConfigError);
}

TEST(SequenceConfig, InlineRelativeWaypointsNeedPriorAbsoluteAndRejectCoordinateOverflow)
{
  const std::string fragment = R"(
  s:
    steps:
      - move:
          absolute: [100, 200, 300, 0]
          waypoints: [{absolute: [10, 20, 30, 0]}, {relative: [0, 0, 1, 0]}]
)";
  EXPECT_NO_THROW(SequenceConfig::from_yaml(document("{}", fragment)));
  EXPECT_THROW(SequenceConfig::from_yaml(document("{}", fragment, "{'0,1': s}")),
    ConfigError);
  for (const std::string entry : {
      "start_sequence", "before_initialization_sequence", "after_initialization_sequence",
      "end_sequence"})
  {
    SCOPED_TRACE(entry);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}", fragment) + entry + ": s\n"),
      ConfigError);
  }
  EXPECT_THROW(SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - waypoint: {absolute: [1.7e308, 0, 0, 0]}
      - move:
          absolute: [1, 2, 3, 0]
          waypoints: [{relative: [1.7e308, 0, 0, 0]}]
)", "{'0,1': s}")), ConfigError);
}

TEST(SequenceConfig, FalseOrOmittedWaypointKeepsOrdinaryMovementBoundaries)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - move: {absolute: [1, 2, 3, 0], waypoint: false}
      - pump: off
      - move: {relative: [0, 0, 1, 0]}
      - endeffector: 1
      - move: {absolute: [4, 5, 6, 0], waypoint: false}
)", "{'0,1': s}"));
  for (const auto & step : config.compile("red", "pick", 0, 1)) {
    EXPECT_FALSE(step.waypoint);
  }
}

TEST(SequenceConfig, RejectsMalformedWaypointFlagsAndAmbiguousMovementModes)
{
  for (const std::string flag : {
      "yes", "no", "on", "off", "True", "FALSE", "1", "0", "0.0", "null", "''",
      "[]", "{}", "[true]", "'$flag'"})
  {
    SCOPED_TRACE(flag);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}",
        "{s: {steps: [{move: {absolute: [1, 2, 3, 0], waypoint: " + flag + "}}]}}")),
      ConfigError);
  }
  for (const std::string move : {
      "{waypoint: true}", "{}",
      "{absolute: [1, 2, 3, 0], relative: [0, 0, 0, 0], waypoint: true}",
      "{absolute: [1, 2, 3, 0], waypoint: true, speed: 1}",
      "{absolute: [1, 2, 3, 0], waypoint: true, waypoint: false}"})
  {
    SCOPED_TRACE(move);
    EXPECT_THROW(SequenceConfig::from_yaml(document(
        "{}", "{s: {steps: [{move: " + move + "}]}}")), ConfigError);
  }
}

TEST(SequenceConfig, RejectsWaypointBeforeNonMovementAfterReferenceExpansion)
{
  for (const std::string operation : {
      "move: {absolute: [1, 2, 3, 0], waypoint: true}",
      "waypoint: {absolute: [1, 2, 3, 0]}"})
  {
    SCOPED_TRACE(operation);
    for (const std::string next : {"{pump: off}", "{wait: 0}", "{endeffector: 1}"}) {
      SCOPED_TRACE(next);
      for (const std::string combined : {
          "{steps: [{call: waypoint}, " + next + "]}",
          std::string{"{extends: waypoint, steps: [{call: next}]}"}})
      {
        SCOPED_TRACE(combined);
        EXPECT_THROW(SequenceConfig::from_yaml(document("{}",
            "{waypoint: {steps: [{" + operation + "}]}, "
            "next: {steps: [" + next + "]}, combined: " + combined + "}",
            "{'0,1': combined}")), ConfigError);
      }
    }
  }
}

TEST(SequenceConfig, TerminalWaypointsAreAllowedOnlyInUnboundFragments)
{
  for (const std::string operation : {
      "move: {absolute: [1, 2, 3, 0], waypoint: true}",
      "waypoint: {absolute: [1, 2, 3, 0]}"})
  {
    SCOPED_TRACE(operation);
    const std::string fragments =
      "\n  waypoint: {steps: [{" + operation + "}]}\n"
      "  empty: {steps: []}\n"
      "  alias: {extends: waypoint, steps: [{call: empty}]}\n";
    EXPECT_NO_THROW(SequenceConfig::from_yaml(document("{}", fragments)));
    EXPECT_THROW(SequenceConfig::from_yaml(document(
        "{}", fragments, "{'0,1': alias}")), ConfigError);
    for (const std::string entry : {
        "start_sequence", "before_initialization_sequence", "after_initialization_sequence",
        "end_sequence"})
    {
      SCOPED_TRACE(entry);
      EXPECT_THROW(SequenceConfig::from_yaml(
          document("{}", fragments) + entry + ": alias\n"), ConfigError);
    }
  }
}

TEST(SequenceConfig, WaypointsCannotContinueAcrossInitializationCommand)
{
  for (const std::string operation : {
      "move: {absolute: [1, 2, 3, 0], waypoint: true}",
      "waypoint: {absolute: [1, 2, 3, 0]}"})
  {
    SCOPED_TRACE(operation);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}",
        "\n  before: {steps: [{" + operation + "}]}\n"
        "  after: {steps: [{move: {absolute: [4, 5, 6, 0]}}]}\n") +
      "before_initialization_sequence: before\nafter_initialization_sequence: after\n"),
      ConfigError);
  }
}

TEST(SequenceConfig, WaypointGroupsPreserveRelativeAnchorAndOverflowValidation)
{
  for (const std::string steps : {
      "[{move: {relative: [0, 0, 1, 0], waypoint: true}}, "
      "{move: {absolute: [1, 2, 3, 0]}}]",
      "[{move: {absolute: [1.7e308, 0, 0, 0], waypoint: true}}, "
      "{move: {relative: [1.7e308, 0, 0, 0]}}]"})
  {
    SCOPED_TRACE(steps);
    EXPECT_THROW(SequenceConfig::from_yaml(document(
        "{}", "{s: {steps: " + steps + "}}", "{'0,1': s}")), ConfigError);
  }
}

TEST(SequenceConfig, MissingOrNullStartSequencePreservesExistingBindings)
{
  const auto yaml = document(
    "{}", "{s: {steps: [{pump: suction}]}}", "{'0,1': s}");
  for (const std::string setting : {"", "start_sequence: null\n"}) {
    SCOPED_TRACE(setting);
    const auto config = SequenceConfig::from_yaml(yaml + setting);
    EXPECT_TRUE(config.compile_start().empty());
    const auto steps = config.compile("red", "pick", 0, 1);
    ASSERT_EQ(steps.size(), 1u);
    EXPECT_EQ(steps[0].type, StepType::PUMP);
    EXPECT_EQ(steps[0].command, 1);
  }
}

TEST(SequenceConfig, RejectsInvalidStartSequenceBeforeExecution)
{
  const auto yaml = document("{}", "{s: {steps: [{pump: off}]}}");
  for (const std::string reference : {"missing", "''", "[]", "{}", "[s]", "{name: s}"}) {
    SCOPED_TRACE(reference);
    EXPECT_THROW(
      SequenceConfig::from_yaml(yaml + "start_sequence: " + reference + "\n"), ConfigError);
  }
  for (const std::string steps : {
      "[{move: {relative: [0, 0, -10, 0]}}]",
      "[{endeffector: 1}, {move: {relative: [0, 0, -10, 0]}}]",
      "[{move: {absolute: [1.7e308, 0, 0, 0]}}, {move: {relative: [1.7e308, 0, 0, 0]}}]"})
  {
    SCOPED_TRACE(steps);
    EXPECT_THROW(
      SequenceConfig::from_yaml(document("{}", "{s: {steps: " + steps + "}}") +
      "start_sequence: s\n"), ConfigError);
  }
}

TEST(SequenceConfig, StartSequenceReloadUsesTheLatestValidSnapshot)
{
  TemporaryConfig file;
  const auto yaml = document("{}", "{s: {steps: [{endeffector: '$width'}]}}") +
    "start_sequence: s\n";
  file.write(yaml + "values: {width: 1}\n");
  auto config = SequenceConfig::load(file.path);
  file.write(yaml + "values: {width: 0}\n");
  EXPECT_EQ(config.compile_start().front().command, 1);
  config = SequenceConfig::load(file.path);
  EXPECT_EQ(config.compile_start().front().command, 0);
  file.write(yaml + "values: {width: 2}\n");
  EXPECT_THROW(config = SequenceConfig::load(file.path), ConfigError);
  EXPECT_EQ(config.compile_start().front().command, 0);
  file.write(document("{}", "{}") + "start_sequence: null\n");
  config = SequenceConfig::load(file.path);
  EXPECT_TRUE(config.compile_start().empty());
}

TEST(SequenceConfig, InitializationHooksSurroundExactlyOneCommand)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  before: {steps: [{pump: off}, {wait: 0.5}]}
  after: {steps: [{endeffector: 1}]}
  ending: {extends: before, steps: [{call: after}]}
)") + "before_initialization_sequence: before\nafter_initialization_sequence: after\n"
    "end_sequence: ending\n");
  const auto initialization = config.compile_initialization();
  ASSERT_EQ(initialization.size(), 4u);
  EXPECT_EQ(initialization[0].type, StepType::PUMP);
  EXPECT_EQ(initialization[1].type, StepType::WAIT);
  EXPECT_EQ(initialization[2].type, StepType::INITIALIZE);
  EXPECT_EQ(initialization[3].type, StepType::ENDEFFECTOR);
  const auto ending = config.compile_end();
  ASSERT_EQ(ending.size(), 3u);
  EXPECT_EQ(ending[0].type, StepType::PUMP);
  EXPECT_EQ(ending[1].type, StepType::WAIT);
  EXPECT_EQ(ending[2].type, StepType::ENDEFFECTOR);
}

TEST(SequenceConfig, OptionalAndExplicitEmptyHooksKeepInitializationCommand)
{
  for (const std::string setting : {
      "", "before_initialization_sequence: null\nafter_initialization_sequence: null\n"
      "end_sequence: null\n",
      "before_initialization_sequence: empty\nafter_initialization_sequence: alias\n"
      "end_sequence: empty\n"})
  {
    SCOPED_TRACE(setting);
    const auto config = SequenceConfig::from_yaml(document(
        "{}", "{empty: {steps: []}, alias: empty}") + setting);
    const auto initialization = config.compile_initialization();
    ASSERT_EQ(initialization.size(), 1u);
    EXPECT_EQ(initialization[0].type, StepType::INITIALIZE);
    EXPECT_TRUE(config.compile_end().empty());
  }
  EXPECT_THROW(SequenceConfig::from_yaml(document(
      "{}", "{empty: {steps: []}}", "{'0,1': empty}")), ConfigError);
}

TEST(SequenceConfig, ValidatesLifecycleReferencesAndIndependentAbsoluteAnchors)
{
  for (const std::string key : {
      "before_initialization_sequence", "after_initialization_sequence", "end_sequence"})
  {
    SCOPED_TRACE(key);
    for (const std::string reference : {"missing", "''", "[]", "{}"}) {
      EXPECT_THROW(SequenceConfig::from_yaml(
          document("{}", "{}") + key + ": " + reference + "\n"), ConfigError);
    }
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}", R"(
  before: {steps: [{move: {absolute: [10, 20, 30, 0]}}]}
  relative: {steps: [{move: {relative: [0, 0, 5, 0]}}]}
)") + key + ": relative\n"), ConfigError);
  }
  const auto yaml = document("{}", R"(
  before: {steps: [{move: {absolute: [10, 20, 30, 0]}}]}
  after: {steps: [{move: {relative: [0, 0, 5, 0]}}]}
)") + "before_initialization_sequence: before\nafter_initialization_sequence: after\n";
  EXPECT_THROW(SequenceConfig::from_yaml(yaml), ConfigError);
}

TEST(SequenceConfig, LifecycleReloadKeepsSnapshotUntilAValidLoad)
{
  TemporaryConfig file;
  const auto yaml = document("{}", R"(
  before: {steps: [{move: {absolute: [10, 20, 30, 0]}}]}
  after:
    steps:
      - move: {absolute: [100, 200, 300, 0]}
      - move: {relative: [0, 0, '$offset', 0]}
  ending: {steps: [{wait: '$offset'}]}
)") + "before_initialization_sequence: before\nafter_initialization_sequence: after\n"
    "end_sequence: ending\n";
  file.write(yaml + "values: {offset: 5}\n");
  auto config = SequenceConfig::load(file.path);
  EXPECT_EQ(config.compile_initialization().back().pose, (Pose{100, 200, 305, 0}));
  file.write(yaml + "values: {offset: 10}\n");
  EXPECT_EQ(config.compile_initialization().back().pose[2], 305);
  EXPECT_EQ(config.compile_end().front().seconds, 5);
  config = SequenceConfig::load(file.path);
  EXPECT_EQ(config.compile_initialization().back().pose[2], 310);
  EXPECT_EQ(config.compile_end().front().seconds, 10);
  file.write(yaml + "values: {offset: -1}\n");
  EXPECT_THROW(config = SequenceConfig::load(file.path), ConfigError);
  EXPECT_EQ(config.compile_initialization().back().pose[2], 310);
  EXPECT_EQ(config.compile_end().front().seconds, 10);
}

TEST(SequenceConfig, InitializationCombinedLimitIncludesTheCommand)
{
  auto make_yaml = [](int before_count, int after_count) {
      std::string sequences = "\n  before:\n    steps:\n";
      for (int i = 0; i < before_count; ++i) {
        sequences += "      - wait: 0\n";
      }
      sequences += "  after:\n    steps:\n";
      for (int i = 0; i < after_count; ++i) {
        sequences += "      - wait: 0\n";
      }
      return document("{}", sequences) + "before_initialization_sequence: before\n"
             "after_initialization_sequence: after\n";
    };
  const auto config = SequenceConfig::from_yaml(make_yaml(5000, 4999));
  EXPECT_EQ(config.compile_initialization().size(), 10000u);
  EXPECT_THROW(SequenceConfig::from_yaml(make_yaml(5000, 5000)), ConfigError);
}

TEST(SequenceConfig, InlineWaypointsCountTowardTheExpandedSequenceLimit)
{
  const auto single = SequenceConfig::from_yaml(document("{}",
      "{s: {steps: [" + movement_with_waypoints(9999) + "]}}", "{'0,1': s}"));
  EXPECT_EQ(single.compile("red", "pick", 0, 1).front().waypoints.size(), 9999u);
  EXPECT_THROW(SequenceConfig::from_yaml(document("{}",
      "{s: {steps: [" + movement_with_waypoints(10000) + "]}}")), ConfigError);

  const std::string fragment =
    "\n  fragment: {steps: [" + movement_with_waypoints(4999) + "]}\n";
  const auto expanded = SequenceConfig::from_yaml(document("{}", fragment +
      "  s: {extends: fragment, steps: [{call: fragment}]}\n", "{'0,1': s}"));
  const auto steps = expanded.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 2u);
  EXPECT_EQ(steps[0].waypoints.size(), 4999u);
  EXPECT_EQ(steps[1].waypoints.size(), 4999u);
  EXPECT_THROW(SequenceConfig::from_yaml(document("{}", fragment +
      "  s: {extends: fragment, steps: [{call: fragment}, {wait: 0}]}\n",
      "{'0,1': s}")), ConfigError);
}

TEST(SequenceConfig, InitializationCombinedLimitIncludesInlineWaypoints)
{
  auto make_yaml = [](std::size_t after_waypoints) {
      return document("{}",
        "\n  before: {steps: [" + movement_with_waypoints(4999) + "]}\n"
        "  after: {steps: [" + movement_with_waypoints(after_waypoints) + "]}\n") +
             "before_initialization_sequence: before\nafter_initialization_sequence: after\n";
    };
  const auto config = SequenceConfig::from_yaml(make_yaml(4998));
  const auto steps = config.compile_initialization();
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_EQ(steps[0].waypoints.size(), 4999u);
  EXPECT_EQ(steps[1].type, StepType::INITIALIZE);
  EXPECT_EQ(steps[2].waypoints.size(), 4998u);
  EXPECT_THROW(SequenceConfig::from_yaml(make_yaml(4999)), ConfigError);
}

TEST(SequenceConfig, MultipleParentsAppendInOrderAndAbsoluteResetsAnchor)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  first: {steps: [{move: {absolute: [1, 2, 3, 0]}}, {pump: release}]}
  second: {steps: [{move: {absolute: [20, 30, 40, 1]}}, {pump: off}]}
  combined:
    extends: [first, second]
    steps:
      - move: {relative: [1, -2, 3, -0.5]}
      - endeffector: 0
      - wait: 0
)", "{}", "{}", "{}", "{'3,1': combined}"));
  const auto steps = config.compile("blue", "place", 3, 1);
  ASSERT_EQ(steps.size(), 7u);
  EXPECT_EQ(steps[0].pose, (Pose{1, 2, 3, 0}));
  EXPECT_EQ(steps[1].command, -1);
  EXPECT_EQ(steps[2].pose, (Pose{20, 30, 40, 1}));
  EXPECT_EQ(steps[3].command, 0);
  EXPECT_EQ(steps[4].pose, (Pose{21, 28, 43, 0.5}));
  EXPECT_EQ(steps[5].command, 0);
  EXPECT_EQ(steps[6].seconds, 0);
}

TEST(SequenceConfig, AllowsUnboundRelativeFragmentsButRejectsUnanchoredBindings)
{
  const std::string fragments = "{common: {steps: [{move: {relative: [0, 0, -10, 0]}}]}}";
  EXPECT_NO_THROW(SequenceConfig::from_yaml(document("{}", fragments)));
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{}", fragments, "{'0,1': common}")),
    ConfigError);
}

TEST(SequenceConfig, SeparatesTeamsAndRejectsMissingDisabledOrInvalidSelections)
{
  const auto config = SequenceConfig::from_yaml(document(
      "{}", "{red: {steps: [{pump: suction}]}, blue: {steps: [{pump: release}]}}",
      "{'0,1': red, '0,2': null}", "{}", "{'0,1': blue}"));
  EXPECT_EQ(config.compile("red", "pick", 0, 1).front().command, 1);
  EXPECT_EQ(config.compile("blue", "pick", 0, 1).front().command, -1);
  EXPECT_THROW(config.compile("red", "pick", 0, 2), ConfigError);
  EXPECT_THROW(config.compile("red", "pick", 0, 3), ConfigError);
  EXPECT_THROW(config.compile("green", "pick", 0, 1), ConfigError);
  EXPECT_THROW(config.compile("red", "PICK", 0, 1), ConfigError);
  EXPECT_THROW(config.compile("red", "pick", 4, 1), ConfigError);
  EXPECT_THROW(config.compile("red", "pick", 0, 0), ConfigError);
  EXPECT_THROW(config.compile("red", "place", 0, 2), ConfigError);
}

TEST(SequenceConfig, RejectsUnknownReferencesAndCyclesEvenWhenUnused)
{
  for (const std::string poses : {
      "{a: missing}", "{a: a}", "{a: b, b: {extends: a, z: 10}}"})
  {
    SCOPED_TRACE(poses);
    EXPECT_THROW(SequenceConfig::from_yaml(document(poses, "{}")), ConfigError);
  }
  for (const std::string sequences : {
      "{a: missing}", "{a: a}", "{a: b, b: {steps: [{call: a}]}}",
      "{a: {extends: b}, b: {extends: a}}",
      "{a: {steps: [{call: missing}]}}",
      "{a: {steps: [{move: {absolute: missing}}]}}"})
  {
    SCOPED_TRACE(sequences);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}", sequences)), ConfigError);
  }
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{}", "{}", "{'0,1': missing}")), ConfigError);
}

TEST(SequenceConfig, RejectsInvalidCommandsUnknownKeysAndAmbiguousOperations)
{
  for (const std::string step : {
      "{initialize: true}", "{initialization: true}", "{wait: -1}", "{wait: .inf}", "{wait: .nan}", "{wait: abc}", "{wait: 1e308}",
      "{pump: true}", "{pump: hold}", "{endeffector: 2}", "{endeffector: -1}",
      "{endeffector: 0.5}", "{pump: off, wait: 1}", "{}", "{pause: 1}",
      "{pump: off, collector_mask: 7}", "{move: {absolute: [1, 2, 3]}}",
      "{move: {absolute: [1, 2, 3, .inf]}}",
      "{move: {relative: [1, 2, .nan, 0]}}",
      "{move: {absolute: [1, 2, 3, 0], relative: [0, 0, 0, 0]}}",
      "{move: {previous: [0, 0, 0, 0]}}"})
  {
    SCOPED_TRACE(step);
    EXPECT_THROW(
      SequenceConfig::from_yaml(document("{}", "{s: {steps: [" + step + "]}}")),
      ConfigError);
  }
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{}", "{s: {steps: [], speed: 1}}")),
    ConfigError);
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{a: [0, 0, 0, 0], b: {extends: a, yaw: 1}}", "{}")),
    ConfigError);
}

TEST(SequenceConfig, RejectsDuplicateKeysMissingStepsAndInvalidBindingIndices)
{
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{a: [0, 0, 0, 0], a: [1, 1, 1, 1]}", "{}")),
    ConfigError);
  for (const std::string sequences : {
      "{s: {extends: []}}", "{s: {}}",
      "{s: {steps: [{pump: off, pump: suction}]}}"})
  {
    SCOPED_TRACE(sequences);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}", sequences)), ConfigError);
  }
  for (const std::string binding : {
      "{'0,1': null, '0,1': null}", "{'0,1': null, '00,01': null}",
      "{'0,0': null}", "{'4,1': null}", "{'0,5': null}", "{'0,1,2': null}",
      "{'0, 1': null}", "{'0': null}", "{'x,1': null}"})
  {
    SCOPED_TRACE(binding);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}", "{}", binding)), ConfigError);
  }
}

TEST(SequenceConfig, RejectsUnsupportedSchemaAndMultipleDocuments)
{
  const auto valid = document("{}", "{}");
  EXPECT_NO_THROW(SequenceConfig::from_yaml(valid));
  EXPECT_THROW(SequenceConfig::from_yaml(""), ConfigError);
  EXPECT_THROW(SequenceConfig::from_yaml("version: 2\n"), ConfigError);
  EXPECT_THROW(SequenceConfig::from_yaml(valid + "collector_mask: 7\n"), ConfigError);
  EXPECT_THROW(SequenceConfig::from_yaml(valid + "---\n" + valid), ConfigError);
  EXPECT_THROW(SequenceConfig::from_yaml(valid + "version: 1\n"), ConfigError);
}

TEST(SequenceConfig, RejectsOverflowWhenRelativeCoordinatesAreExpanded)
{
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - move: {absolute: [1.7e308, 0, 0, 0]}
      - move: {relative: [1.7e308, 0, 0, 0]}
)", "{'0,1': s}")), ConfigError);
}

TEST(SequenceConfig, ReusesNumericAliasesAcrossPosesOverridesAndCommands)
{
  const auto config = SequenceConfig::from_yaml(document(R"(
  base: ['$x', 200, '$above_z', '$angle']
  inherited: {extends: base, z: '$higher_z'}
)", R"(
  s:
    steps:
      - move: {absolute: inherited}
      - move: {relative: [0, 0, '$lower', '$rotate']}
      - wait: '$grip_wait'
      - endeffector: '$rotate_command'
)", "{'0,1': s}") + R"(
values:
  x: 100
  above_z: 300
  higher_z: '$shared_height'
  shared_height: 400
  angle: 0.5
  lower: -50
  rotate: 0.25
  grip_wait: '$common_wait'
  common_wait: 0.2
  rotate_command: 1
)");
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 4u);
  EXPECT_EQ(steps[0].pose, (Pose{100, 200, 400, 0.5}));
  EXPECT_EQ(steps[1].pose, (Pose{100, 200, 350, 0.75}));
  EXPECT_DOUBLE_EQ(steps[2].seconds, 0.2);
  EXPECT_EQ(steps[3].command, 1);
}

TEST(SequenceConfig, RejectsBadNumericReferencesEvenWhenUnused)
{
  for (const std::string values : {
      "{a: '$missing'}", "{a: '$a'}", "{a: '$b', b: '$a'}", "{a: .nan}",
      "{a: .inf}", "{a: '$'}", "{a: b}", "{a: []}", "{a: 1, a: 2}"})
  {
    SCOPED_TRACE(values);
    EXPECT_THROW(
      SequenceConfig::from_yaml(document("{}", "{}") + "values: " + values),
      ConfigError);
  }
  for (const std::string step : {
      "{wait: '$negative'}", "{endeffector: '$negative'}", "{wait: '$missing'}",
      "{move: {absolute: [0, 0, '$missing', 0]}}"})
  {
    SCOPED_TRACE(step);
    EXPECT_THROW(
      SequenceConfig::from_yaml(document("{}", "{s: {steps: [" + step + "]}}") +
      "values: {negative: -1}\n"), ConfigError);
  }
}

TEST(SequenceConfig, SupportsNativeYamlScalarAnchors)
{
  const auto config = SequenceConfig::from_yaml(R"(
version: 1
values:
  height: &height 300
poses:
  a: [100, 200, *height, 0]
sequences:
  s: {steps: [{move: {absolute: a}}]}
bindings:
  red: {pick: {'0,1': s}, place: {}}
  blue: {pick: {}, place: {}}
)");
  EXPECT_EQ(config.compile("red", "pick", 0, 1).front().pose[2], 300);
}

TEST(SequenceConfig, BoundsWaitDurationIncludingReferencedValues)
{
  const auto config = SequenceConfig::from_yaml(document(
      "{}", "{s: {steps: [{wait: 0}, {wait: '$day'}]}}", "{'0,1': s}") +
    "values: {day: 86400}\n");
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 2u);
  EXPECT_DOUBLE_EQ(steps[0].seconds, 0);
  EXPECT_DOUBLE_EQ(steps[1].seconds, MAX_DURATION_SEC);
  for (const std::string duration : {"86400.001", "1e308"}) {
    SCOPED_TRACE(duration);
    EXPECT_THROW(
      SequenceConfig::from_yaml(document("{}", "{s: {steps: [{wait: '$too_long'}]}}") +
      "values: {too_long: " + duration + "}\n"), ConfigError);
  }
}

TEST(SequenceConfig, ReloadsOnlyOnLoadAndKeepsPreviousSnapshotAfterFailedReload)
{
  TemporaryConfig file;
  const auto initial = document(
    "{p: [10, 20, 30, 0]}", "{s: {steps: [{move: {absolute: p}}]}}", "{'0,1': s}");
  file.write(initial);
  auto config = SequenceConfig::load(file.path);
  file.write(document(
      "{p: [40, 50, 60, 1]}", "{s: {steps: [{move: {absolute: p}}]}}", "{'0,1': s}"));
  EXPECT_EQ(config.compile("red", "pick", 0, 1).front().pose, (Pose{10, 20, 30, 0}));
  config = SequenceConfig::load(file.path);
  EXPECT_EQ(config.compile("red", "pick", 0, 1).front().pose, (Pose{40, 50, 60, 1}));
  file.write("version: [broken\n");
  EXPECT_THROW(config = SequenceConfig::load(file.path), ConfigError);
  EXPECT_EQ(config.compile("red", "pick", 0, 1).front().pose, (Pose{40, 50, 60, 1}));
  std::remove(file.path.c_str());
  EXPECT_THROW(SequenceConfig::load(file.path), ConfigError);
}

}  // namespace
}  // namespace catchrobo2026_sequence
