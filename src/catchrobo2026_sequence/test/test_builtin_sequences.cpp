#include "catchrobo2026_sequence/sequence_config.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace catchrobo2026_sequence
{
namespace
{

std::string key(int first, int second)
{
  return std::to_string(first) + "," + std::to_string(second);
}

double value(const YAML::Node & yaml, std::string name)
{
  for (int depth = 0; depth < 64; ++depth) {
    const auto entry = yaml["values"][name];
    const auto text = entry.as<std::string>();
    if (!text.empty() && text.front() == '$') {
      name = text.substr(1);
    } else {
      return entry.as<double>();
    }
  }
  throw std::runtime_error("value reference cycle");
}

void expect_same_steps(const std::vector<Step> & actual, const std::vector<Step> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    SCOPED_TRACE(index);
    EXPECT_EQ(actual[index].type, expected[index].type);
    EXPECT_EQ(actual[index].waypoint, expected[index].waypoint);
    EXPECT_EQ(actual[index].waypoints, expected[index].waypoints);
    for (std::size_t axis = 0; axis < actual[index].pose.size(); ++axis) {
      EXPECT_DOUBLE_EQ(actual[index].pose[axis], expected[index].pose[axis]);
    }
    EXPECT_EQ(actual[index].command, expected[index].command);
    EXPECT_DOUBLE_EQ(actual[index].seconds, expected[index].seconds);
  }
}

template<typename Callback>
void for_each_binding(Callback callback)
{
  for (const std::string team : {"red", "blue"}) {
    for (const std::string kind : {"pick", "place"}) {
      for (int first = 0; first < 4; ++first) {
        const int begin = kind == "pick" ? 1 : 0;
        const int end = kind == "pick" ? 5 : 2;
        for (int second = begin; second < end; ++second) {
          SCOPED_TRACE(team + " " + kind + " " + key(first, second));
          callback(team, kind, first, second);
        }
      }
    }
  }
}

TEST(BuiltinSequences, InitializationOnlySendsCommandAndEndPassesWaypointToLifecyclePose)
{
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  const auto steps = config.compile_initialization();
  ASSERT_EQ(steps.size(), 1u);
  EXPECT_EQ(steps[0].type, StepType::INITIALIZE);
  const auto ending = config.compile_end();
  ASSERT_EQ(ending.size(), 2u);
  EXPECT_EQ(ending[0].type, StepType::MOVE);
  EXPECT_EQ(ending[0].pose, (Pose{675, 200, 300, 0}));
  EXPECT_TRUE(ending[0].waypoint);
  EXPECT_EQ(ending[1].type, StepType::MOVE);
  EXPECT_EQ(ending[1].pose, (Pose{670, -110, 220, 0}));
  EXPECT_FALSE(ending[1].waypoint);
}

TEST(BuiltinSequences, EveryUiPositionIsConfigured)
{
  const auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for (const std::string team : {"red", "blue"}) {
    SCOPED_TRACE(team);
    ASSERT_EQ(yaml["bindings"][team]["pick"].size(), 16u);
    ASSERT_EQ(yaml["bindings"][team]["place"].size(), 8u);
    for (int row = 0; row < 4; ++row) {
      for (int column = 1; column < 5; ++column) {
        SCOPED_TRACE(key(row, column));
        const auto steps = config.compile(team, "pick", row, column);
        ASSERT_FALSE(steps.empty());
        EXPECT_EQ(steps.front().type, StepType::MOVE);
      }
    }
    for (int box = 0; box < 4; ++box) {
      for (int column = 0; column < 2; ++column) {
        SCOPED_TRACE(key(box, column));
        const auto steps = config.compile(team, "place", box, column);
        ASSERT_FALSE(steps.empty());
        EXPECT_EQ(steps.front().type, StepType::MOVE);
      }
    }
  }
}

TEST(BuiltinSequences, StartSequenceSelectsPlaceWidthAndTracksItsConfiguredValue)
{
  auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  const auto steps = config.compile_start();
  ASSERT_EQ(steps.size(), 1u);
  EXPECT_EQ(steps[0].type, StepType::ENDEFFECTOR);
  EXPECT_EQ(steps[0].command, value(yaml, "place_endeffector_command"));
  yaml["values"]["place_endeffector_command"] = 1 - steps[0].command;
  const auto changed = SequenceConfig::from_yaml(YAML::Dump(yaml)).compile_start();
  ASSERT_EQ(changed.size(), 1u);
  EXPECT_EQ(changed[0].type, StepType::ENDEFFECTOR);
  EXPECT_EQ(changed[0].command, 1 - steps[0].command);
}

TEST(BuiltinSequences, PickupBindingsAndExpandedStepsAreSharedBetweenTeams)
{
  const auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for (int row = 0; row < 4; ++row) {
    for (int column = 1; column < 5; ++column) {
      const auto binding = key(row, column);
      SCOPED_TRACE(binding);
      EXPECT_EQ(
        yaml["bindings"]["red"]["pick"][binding].as<std::string>(),
        yaml["bindings"]["blue"]["pick"][binding].as<std::string>());
      expect_same_steps(
        config.compile("red", "pick", row, column),
        config.compile("blue", "pick", row, column));
    }
  }
}

TEST(BuiltinSequences, PlacementBindingsSelectDifferentTeamApproaches)
{
  const auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for (int box = 0; box < 4; ++box) {
    for (int column = 0; column < 2; ++column) {
      const auto binding = key(box, column);
      SCOPED_TRACE(binding);
      EXPECT_NE(
        yaml["bindings"]["red"]["place"][binding].as<std::string>(),
        yaml["bindings"]["blue"]["place"][binding].as<std::string>());
      const auto red = config.compile("red", "place", box, column);
      const auto blue = config.compile("blue", "place", box, column);
      ASSERT_GE(red.size(), 2u);
      ASSERT_GE(blue.size(), 2u);
      EXPECT_EQ(red.front().type, StepType::MOVE);
      EXPECT_EQ(blue.front().type, StepType::MOVE);
      EXPECT_EQ(red.front().pose, (Pose{150, 0, 400, 3.14159265358979}));
      EXPECT_EQ(blue.front().pose, (Pose{1200, 0, 400, 3.14159265358979}));
      EXPECT_NE(red.front().pose, blue.front().pose);
      EXPECT_TRUE(red.front().waypoint);
      EXPECT_TRUE(blue.front().waypoint);
      EXPECT_EQ(red[1].type, StepType::MOVE);
      EXPECT_EQ(blue[1].type, StepType::MOVE);
      EXPECT_FALSE(red[1].waypoint);
      EXPECT_FALSE(blue[1].waypoint);
    }
  }
}

TEST(BuiltinSequences, AllBindingsUseTheConfiguredPumpAndWaypointOrder)
{
  const auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for_each_binding([&](const std::string & team, const std::string & kind, int first, int second) {
      const auto steps = config.compile(team, kind, first, second);
      ASSERT_EQ(steps.size(), kind == "pick" ? 5u : 7u);
      const auto anchor_index = kind == "pick" ? 0u : 1u;
      EXPECT_EQ(steps[0].type, StepType::MOVE);
      EXPECT_EQ(steps[0].waypoint, kind == "place");
      EXPECT_EQ(steps[anchor_index].type, StepType::MOVE);
      EXPECT_FALSE(steps[anchor_index].waypoint);
      EXPECT_EQ(steps[anchor_index + 1].type, StepType::PUMP);
      EXPECT_EQ(steps[anchor_index + 1].command, kind == "pick" ? 1 : -1);
      EXPECT_EQ(steps[anchor_index + 2].type, StepType::MOVE);
      EXPECT_EQ(steps[anchor_index + 3].type, StepType::MOVE);
      if (kind == "place") {
        EXPECT_EQ(steps[anchor_index + 4].type, StepType::PUMP);
        EXPECT_EQ(steps[anchor_index + 4].command, 0);
      }
      EXPECT_EQ(steps.back().type, StepType::ENDEFFECTOR);
      EXPECT_EQ(
        steps.back().command,
        value(yaml, kind == "pick" ? "place_endeffector_command" : "pick_endeffector_command"));
    });
}

TEST(BuiltinSequences, BothRelativeOffsetsUseTheFixedApproachAnchor)
{
  const auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for_each_binding([&](const std::string & team, const std::string & kind, int first, int second) {
      const auto steps = config.compile(team, kind, first, second);
      ASSERT_EQ(steps.size(), kind == "pick" ? 5u : 7u);
      const auto anchor_index = kind == "pick" ? 0u : 1u;
      for (const auto relative_index : {2u, 3u}) {
        const auto index = anchor_index + relative_index;
        const std::string offset = kind + (relative_index == 2 ? "_approach_dz" : "_retreat_dz");
        for (const auto axis : {0u, 1u, 3u}) {
          EXPECT_DOUBLE_EQ(steps[index].pose[axis], steps[anchor_index].pose[axis]);
        }
        EXPECT_DOUBLE_EQ(steps[index].pose[2], steps[anchor_index].pose[2] + value(yaml, offset));
      }
    });
}

TEST(BuiltinSequences, OneHeightEditMovesOnlyTheSelectedRegionAndItsRelativeWaypoints)
{
  const auto before = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for (const std::string name : {"work_above_z", "common_work_above_z", "place_above_z"}) {
    SCOPED_TRACE(name);
    auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
    const double original_height = value(yaml, name);
    constexpr double delta = 137.0;
    yaml["values"][name] = original_height + delta;
    const auto after = SequenceConfig::from_yaml(YAML::Dump(yaml));
    for_each_binding([&](const std::string & team, const std::string & kind, int first, int second) {
        auto expected = before.compile(team, kind, first, second);
        const bool changed =
          (name == "work_above_z" && kind == "pick" && first < 3) ||
          (name == "common_work_above_z" && kind == "pick" && first == 3) ||
          (name == "place_above_z" && kind == "place");
        if (changed) {
          const auto anchor_index = kind == "pick" ? 0u : 1u;
          for (std::size_t index = anchor_index; index < expected.size(); ++index) {
            auto & step = expected[index];
            if (step.type == StepType::MOVE) {
              step.pose[2] += delta;
            }
          }
        }
        expect_same_steps(after.compile(team, kind, first, second), expected);
      });
  }
}

TEST(BuiltinSequences, OneRelativeOffsetEditChangesOnlyItsWaypointAcrossTheMatchingBindings)
{
  const auto before = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for (const std::string changed_kind : {"pick", "place"}) {
    for (const auto index : {2u, 3u}) {
      const std::string name = changed_kind +
        (index == 2 ? "_approach_dz" : "_retreat_dz");
      SCOPED_TRACE(name);
      auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
      constexpr double delta = 83.0;
      yaml["values"][name] = value(yaml, name) + delta;
      const auto after = SequenceConfig::from_yaml(YAML::Dump(yaml));
      for_each_binding([&](const std::string & team, const std::string & kind, int first, int second) {
          auto expected = before.compile(team, kind, first, second);
          ASSERT_EQ(expected.size(), kind == "pick" ? 5u : 7u);
          if (kind == changed_kind) {
            const auto anchor_index = kind == "pick" ? 0u : 1u;
            expected[anchor_index + index].pose[2] += delta;
          }
          expect_same_steps(after.compile(team, kind, first, second), expected);
        });
    }
  }
}

TEST(BuiltinSequences, OneCommonSequenceEditChangesEveryMatchingBindingOnly)
{
  const auto before = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  for (const std::string changed_kind : {"pick", "place"}) {
    SCOPED_TRACE(changed_kind);
    auto yaml = YAML::LoadFile(SEQUENCE_CONFIG_PATH);
    YAML::Node extra;
    extra["wait"] = 0.375;
    yaml["sequences"][changed_kind + "_common"]["steps"].push_back(extra);
    const auto after = SequenceConfig::from_yaml(YAML::Dump(yaml));
    for_each_binding([&](const std::string & team, const std::string & kind, int first, int second) {
        auto expected = before.compile(team, kind, first, second);
        if (kind == changed_kind) {
          Step wait;
          wait.type = StepType::WAIT;
          wait.seconds = 0.375;
          expected.push_back(wait);
        }
        expect_same_steps(after.compile(team, kind, first, second), expected);
      });
  }
}

}  // namespace
}  // namespace catchrobo2026_sequence
