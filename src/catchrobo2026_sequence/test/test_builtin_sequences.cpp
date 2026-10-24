#include "catchrobo2026_sequence/sequence_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace catchrobo2026_sequence
{
TEST(BuiltinSequences, FestivalRetainsInitializationWithoutServo)
{
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  const auto init = config.compile_initialization();
  ASSERT_EQ(init.size(), 1u);
  EXPECT_EQ(init[0].type, StepType::INITIALIZE);
  const auto startup = config.compile_start();
  EXPECT_TRUE(startup.empty());
}

TEST(BuiltinSequences, EndingRetainsPositionsWithoutRotationSteps)
{
  const auto ending = SequenceConfig::load(SEQUENCE_CONFIG_PATH).compile_end();
  ASSERT_EQ(ending.size(), 2u);
  EXPECT_EQ(ending[0].type, StepType::MOVE);
  EXPECT_EQ(ending[0].pose, (Pose{675, 400, 300, 0}));
  EXPECT_EQ(ending[1].type, StepType::MOVE);
  EXPECT_EQ(ending[1].pose, (Pose{675, -110, 220, 0}));
}

TEST(BuiltinSequences, GameBindingsAreRemoved)
{
  const auto config = SequenceConfig::load(SEQUENCE_CONFIG_PATH);
  EXPECT_TRUE(config.configured_bindings("both").empty());
  EXPECT_THROW(config.compile("red", "pick", 0, 1), ConfigError);
  EXPECT_THROW(config.compile("blue", "place", 0, 0), ConfigError);
}
}  // namespace catchrobo2026_sequence
