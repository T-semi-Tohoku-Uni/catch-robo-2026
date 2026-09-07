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
      "{wait: -1}", "{wait: .inf}", "{wait: .nan}", "{wait: abc}", "{wait: 1e308}",
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

TEST(SequenceConfig, RejectsDuplicateKeysEmptySequencesAndInvalidBindingIndices)
{
  EXPECT_THROW(
    SequenceConfig::from_yaml(document("{a: [0, 0, 0, 0], a: [1, 1, 1, 1]}", "{}")),
    ConfigError);
  for (const std::string sequences : {
      "{s: {steps: []}}", "{s: {extends: []}}", "{s: {}}",
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
