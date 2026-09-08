#include "catchrobo2026_sequence/sequence_config.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
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

TEST(SequenceConfig, CompilesBoundedSuctionRetriesAndFailureBranch)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - move: {absolute: [10, 20, 30, 0]}
      - for:
          max_iterations: '$attempts'
          steps:
            - suction_check: {start: {timeout: '$timeout'}}
            - pump: suction
            - move: {relative: [0, 0, -5, 0]}
            - suction_check: wait
            - if:
                condition: suction_success
                then: [{break: true}]
      - if:
          condition: suction_failure
          then: [{fail: 'suction retry limit reached'}]
      - move: {relative: [0, 0, 5, 0]}
)", "{'0,1': s}") + "values: {attempts: 3, timeout: 0.75}\n");
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 22u);
  for (std::size_t iteration = 0; iteration < 3; ++iteration) {
    const auto start = 1 + iteration * 6;
    EXPECT_EQ(steps[start].type, StepType::SUCTION_CHECK_START);
    EXPECT_DOUBLE_EQ(steps[start].seconds, 0.75);
    EXPECT_EQ(steps[start + 1].type, StepType::PUMP);
    EXPECT_EQ(steps[start + 2].pose, (Pose{10, 20, 25, 0}));
    EXPECT_EQ(steps[start + 3].type, StepType::SUCTION_CHECK_WAIT);
    EXPECT_EQ(steps[start + 4].type, StepType::IF_SUCTION);
    EXPECT_TRUE(steps[start + 4].condition_success);
    EXPECT_EQ(steps[start + 4].jump_index, start + 6);
    EXPECT_EQ(steps[start + 5].type, StepType::JUMP);
    EXPECT_EQ(steps[start + 5].jump_index, 19u);
  }
  EXPECT_EQ(steps[19].type, StepType::IF_SUCTION);
  EXPECT_FALSE(steps[19].condition_success);
  EXPECT_EQ(steps[19].jump_index, 21u);
  EXPECT_EQ(steps[20].type, StepType::FAIL);
  EXPECT_EQ(steps[20].message, "suction retry limit reached");
  EXPECT_EQ(steps[21].pose, (Pose{10, 20, 35, 0}));
}

TEST(SequenceConfig, RelocatesConditionalJumpsAcrossCallsInheritanceAndHooks)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  check:
    steps:
      - suction_check: {start: {timeout: 1}}
      - suction_check: wait
      - if:
          condition: suction_success
          then: [{pump: suction}]
          else: [{pump: release}]
  combined:
    extends: check
    steps: [{wait: 0}, {call: check}, {wait: 0}]
)") + "before_initialization_sequence: check\n"
    "after_initialization_sequence: combined\n");
  const auto steps = config.compile_initialization();
  ASSERT_EQ(steps.size(), 21u);
  EXPECT_EQ(steps[2].jump_index, 5u);
  EXPECT_EQ(steps[4].jump_index, 6u);
  EXPECT_EQ(steps[6].type, StepType::INITIALIZE);
  EXPECT_EQ(steps[9].jump_index, 12u);
  EXPECT_EQ(steps[11].jump_index, 13u);
  EXPECT_EQ(steps[16].jump_index, 19u);
  EXPECT_EQ(steps[18].jump_index, 20u);
}

TEST(SequenceConfig, NestedBreakExitsOnlyTheInnermostLoop)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - for:
          max_iterations: 2
          steps:
            - pump: off
            - for:
                max_iterations: 3
                steps:
                  - pump: suction
                  - break: true
            - pump: release
      - wait: 0
)", "{'0,1': s}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 17u);
  std::vector<int> commands;
  for (std::size_t index = 0; index < steps.size();) {
    const auto & step = steps[index];
    if (step.type == StepType::PUMP) {
      commands.push_back(step.command);
    }
    if (step.type == StepType::JUMP) {
      ASSERT_GT(step.jump_index, index);
      index = step.jump_index;
    } else {
      ++index;
    }
  }
  EXPECT_EQ(commands, (std::vector<int>{0, 1, -1, 0, 1, -1}));
  EXPECT_EQ(steps[2].jump_index, 7u);
  EXPECT_EQ(steps[10].jump_index, 15u);
}

TEST(SequenceConfig, RejectsInvalidSuctionCheckOrderingOnAnyReachablePath)
{
  const std::string start = "{suction_check: {start: {timeout: 1}}}";
  const std::string wait = "{suction_check: wait}";
  const std::string condition = "{if: {condition: suction_success, then: []}}";
  for (const std::string & steps : {
      wait, start, start + ", " + start + ", " + wait,
      start + ", " + wait + ", " + wait, condition,
      start + ", " + condition + ", " + wait,
      start + ", " + wait + ", " + start + ", " + condition + ", " + wait,
      start + ", {for: {max_iterations: 2, steps: [" + wait + "]}}",
      start + ", " + wait + ", {if: {condition: suction_success, then: [" +
      start + "]}}, " + wait,
      start + ", " + wait + ", {if: {condition: suction_success, then: [" + start + "]}}",
      "{for: {max_iterations: 2, steps: [" + start + ", {break: true}, " + wait + "]}}"})
  {
    SCOPED_TRACE(steps);
    EXPECT_THROW(SequenceConfig::from_yaml(document(
        "{}", "{s: {steps: [" + steps + "]}}", "{'0,1': s}")), ConfigError);
  }
  EXPECT_THROW(SequenceConfig::from_yaml(document("{}",
      "{before: {steps: [" + start + "]}, after: {steps: [" + wait + "]}}") +
    "before_initialization_sequence: before\nafter_initialization_sequence: after\n"),
    ConfigError);
}

TEST(SequenceConfig, PreservesCompletedResultAcrossBranchesAndFragments)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  start_check: {steps: [{suction_check: {start: {timeout: 1}}}]}
  finish_check: {steps: [{suction_check: wait}]}
  s:
    steps:
      - call: start_check
      - call: finish_check
      - if: {condition: suction_failure, then: [{wait: 0}], else: [{pump: suction}]}
      - if: {condition: suction_success, then: [{wait: 0}], else: [{pump: release}]}
)", "{'0,1': s}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 10u);
  EXPECT_FALSE(steps[2].condition_success);
  EXPECT_TRUE(steps[6].condition_success);
}

TEST(SequenceConfig, RejectsAmbiguousAnchorsAfterBranchesOrLoopBreaks)
{
  for (const std::string body : {
      "{if: {condition: suction_success, then: [{move: {absolute: [50, 60, 70, 0]}}]}}",
      "{for: {max_iterations: 2, steps: ["
      "{if: {condition: suction_success, then: [{break: true}]}}, "
      "{move: {absolute: [50, 60, 70, 0]}}]}}"})
  {
    SCOPED_TRACE(body);
    const auto yaml = document("{}", "{s: {steps: ["
      "{move: {absolute: [10, 20, 30, 0]}}, "
      "{suction_check: {start: {timeout: 1}}}, {suction_check: wait}, " + body +
      ", {move: {relative: [0, 0, 5, 0]}}]}}", "{'0,1': s}");
    EXPECT_THROW(SequenceConfig::from_yaml(yaml), ConfigError);
  }
  EXPECT_THROW(SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - suction_check: {start: {timeout: 1}}
      - suction_check: wait
      - if:
          condition: suction_success
          then: [{move: {absolute: [10, 20, 30, 0]}}]
      - move: {relative: [0, 0, 5, 0]}
)", "{'0,1': s}")), ConfigError);
}

TEST(SequenceConfig, BranchAnchorsCanBeSelectedByResultOrResetWithAbsoluteMove)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - suction_check: {start: {timeout: 1}}
      - suction_check: wait
      - if:
          condition: suction_success
          then: [{move: {absolute: [10, 20, 30, 0]}}]
          else: [{move: {absolute: [50, 60, 70, 0]}}]
      - if:
          condition: suction_success
          then: [{move: {relative: [0, 0, 5, 0]}}]
          else: [{move: {relative: [0, 0, -5, 0]}}]
      - move: {absolute: [100, 200, 300, 0]}
      - move: {relative: [0, 0, 5, 0]}
)", "{'0,1': s}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 12u);
  EXPECT_EQ(steps[7].pose, (Pose{10, 20, 35, 0}));
  EXPECT_EQ(steps[9].pose, (Pose{50, 60, 65, 0}));
  EXPECT_EQ(steps[11].pose, (Pose{100, 200, 305, 0}));
}

TEST(SequenceConfig, ResolvesRelativeAnchorsForEachUnrolledIteration)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - move: {absolute: [10, 20, 30, 0]}
      - for:
          max_iterations: 2
          steps:
            - move: {relative: [0, 0, 5, 0]}
            - move: {absolute: [50, 60, 70, 0]}
      - move: {relative: [0, 0, -5, 0]}
)", "{'0,1': s}"));
  const auto steps = config.compile("red", "pick", 0, 1);
  ASSERT_EQ(steps.size(), 6u);
  EXPECT_EQ(steps[1].pose, (Pose{10, 20, 35, 0}));
  EXPECT_EQ(steps[3].pose, (Pose{50, 60, 75, 0}));
  EXPECT_EQ(steps[5].pose, (Pose{50, 60, 65, 0}));
}

TEST(SequenceConfig, RejectsInvalidControlFlowSyntaxEvenWhenUnused)
{
  for (const std::string step : {
      "{suction_check: start}", "{suction_check: null}", "{suction_check: []}",
      "{suction_check: {}}", "{suction_check: {start: {}}}",
      "{suction_check: {start: {timeout: 0}}}", "{suction_check: {start: {timeout: -1}}}",
      "{suction_check: {start: {timeout: 86400.1}}}",
      "{suction_check: {start: {timeout: .inf}}}",
      "{suction_check: {start: {timeout: '$missing'}}}",
      "{suction_check: {start: {timeout: 1, collector_mask: 7}}}",
      "{suction_check: {start: {timeout: 1}, wait: true}}",
      "{if: {condition: pump_on, then: []}}", "{if: {condition: suction_success}}",
      "{if: {condition: suction_success, then: null}}",
      "{if: {condition: suction_success, then: [], else: null}}",
      "{if: {condition: suction_success, then: [], otherwise: []}}",
      "{for: {max_iterations: 0, steps: []}}", "{for: {max_iterations: -1, steps: []}}",
      "{for: {max_iterations: 1.5, steps: []}}",
      "{for: {max_iterations: 10001, steps: []}}",
      "{for: {max_iterations: .nan, steps: []}}",
      "{for: {max_iterations: '$missing', steps: []}}",
      "{for: {max_iterations: 1}}", "{for: {steps: []}}",
      "{for: {max_iterations: 1, steps: [], count: 1}}",
      "{break: true}", "{for: {max_iterations: 1, steps: [{break: false}]}}",
      "{for: {max_iterations: 1, steps: [{break: 1}]}}",
      "{fail: ''}", "{fail: []}", "{fail: null}"})
  {
    SCOPED_TRACE(step);
    EXPECT_THROW(SequenceConfig::from_yaml(document(
        "{}", "{s: {steps: [" + step + "]}}")), ConfigError);
  }
}

TEST(SequenceConfig, BoundsLoopExpansionAndNestedCalls)
{
  const auto config = SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - for:
          max_iterations: 100
          steps:
            - for: {max_iterations: 100, steps: [{wait: 0}]}
)", "{'0,1': s}"));
  EXPECT_EQ(config.compile("red", "pick", 0, 1).size(), 10000u);
  EXPECT_NO_THROW(SequenceConfig::from_yaml(document("{}", R"(
  s:
    steps:
      - for:
          max_iterations: 10000
          steps:
            - for: {max_iterations: 10000, steps: []}
)")));
  for (const std::string sequences : {
      "{s: {steps: [{for: {max_iterations: 10000, steps: [{wait: 0}, {wait: 0}]}}]}}",
      "{s: {steps: [{for: {max_iterations: 2, steps: [{call: s}]}}]}}",
      "{s: {steps: [{if: {condition: suction_success, then: [{call: s}]}}]}}",
      "{s: {steps: [{for: {max_iterations: 1, steps: [{call: missing}]}}]}}"})
  {
    SCOPED_TRACE(sequences);
    EXPECT_THROW(SequenceConfig::from_yaml(document("{}", sequences)), ConfigError);
  }
  std::string nested = "{wait: 0}";
  for (int depth = 0; depth < 64; ++depth) {
    nested = "{for: {max_iterations: 1, steps: [" + nested + "]}}";
  }
  EXPECT_THROW(SequenceConfig::from_yaml(document(
      "{}", "{s: {steps: [" + nested + "]}}")), ConfigError);
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
