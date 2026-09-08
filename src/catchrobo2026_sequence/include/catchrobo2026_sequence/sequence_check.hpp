#ifndef CATCHROBO2026_SEQUENCE__SEQUENCE_CHECK_HPP_
#define CATCHROBO2026_SEQUENCE__SEQUENCE_CHECK_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "catchrobo2026_sequence/sequence_config.hpp"

namespace catchrobo2026_sequence
{

using CheckJoints = std::array<float, 4>;

enum class CheckStatus {FEASIBLE, INFEASIBLE, UNKNOWN};

struct CheckDiagnostic
{
  std::string severity;
  std::string code;
  std::string message;
  std::size_t step_index{0};
  std::size_t route_index{0};
  std::size_t sample_index{0};
};

struct SequenceCheckResult
{
  CheckStatus status{CheckStatus::FEASIBLE};
  std::string message;
  std::vector<CheckDiagnostic> diagnostics;
  std::optional<CheckJoints> final_joints;
  double phi_travel{0.0};
  double max_wrist_step{0.0};
  std::size_t route_count{0};
  std::size_t sample_count{0};
};

struct SequenceCheckOptions
{
  std::optional<CheckJoints> after_initialization_joints;
};

// Checks ideal commanded paths without creating a ROS context or sending commands.
SequenceCheckResult check_sequence_steps(
  const std::vector<Step> & steps,
  const std::optional<CheckJoints> & initial_joints,
  const SequenceCheckOptions & options = {});

}  // namespace catchrobo2026_sequence

#endif  // CATCHROBO2026_SEQUENCE__SEQUENCE_CHECK_HPP_
