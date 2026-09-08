#ifndef CATCHROBO2026_SEQUENCE__SEQUENCE_CONFIG_HPP_
#define CATCHROBO2026_SEQUENCE__SEQUENCE_CONFIG_HPP_

#include <array>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace catchrobo2026_sequence
{

using Pose = std::array<double, 4>;
inline constexpr double MAX_DURATION_SEC = 86400.0;

enum class StepType
{
  MOVE, PUMP, ENDEFFECTOR, WAIT, INITIALIZE, SEQUENCE_START, SEQUENCE_END,
  PHI_TRAVEL_START, PHI_TRAVEL_END,
  ROTATION_START = SEQUENCE_START, ROTATION_END = SEQUENCE_END
};

struct Step
{
  StepType type{StepType::MOVE};
  Pose pose{};
  bool waypoint{false};
  std::vector<Pose> waypoints;
  int command{0};
  double seconds{0.0};
  bool rotation_group{false};
  std::optional<double> max_phi_travel;
};

struct PreparedSequence
{
  std::vector<Step> steps;
  std::vector<Pose> deferred_waypoints;
  std::size_t consumed_pending_waypoints{0};
};

struct PhiTravelInterval
{
  std::size_t start_target, end_target;
  double max_phi_travel;
};

// Target indices count inline and standalone waypoints as well as MOVE endpoints.
std::vector<PhiTravelInterval> collect_phi_travel_intervals(
  const std::vector<Step> & steps, std::size_t group_start, std::size_t group_end);

// Accepts compiled steps; deferred poses retain their original absolute anchor.
PreparedSequence prepare_sequence(
  std::vector<Step> steps, const std::vector<Pose> & pending = {});

struct SequenceBinding
{
  std::string team;
  std::string kind;
  int index1;
  int index2;
  std::string sequence;
};

class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class SequenceConfig
{
public:
  static SequenceConfig load(const std::string & path);
  static SequenceConfig from_yaml(const std::string & yaml);

  std::vector<Step> compile(
    const std::string & team, const std::string & kind, int index1, int index2) const;
  std::vector<Step> compile_start() const;
  std::vector<Step> compile_initialization() const;
  std::vector<Step> compile_end() const;
  std::vector<SequenceBinding> configured_bindings(const std::string & team) const;
  std::vector<Step> compile_named(const std::string & name) const;
  Pose named_pose(const std::string & name) const;
  std::optional<double> route_timeout_sec() const {return route_timeout_sec_;}

private:
  struct RawPose
  {
    Pose pose{};
    bool relative{false};
  };

  struct RawStep
  {
    Step step;
    bool relative{false};
    std::vector<RawPose> waypoints;
  };

  using BindingKey = std::tuple<std::string, std::string, int, int>;
  std::vector<Step> compile_sequence(
    const std::string & name, const std::string & where,
    bool allow_deferred_waypoints = true) const;

  std::map<std::string, std::vector<RawStep>> sequences_;
  std::map<std::string, Pose> poses_;
  std::map<BindingKey, std::string> bindings_;
  std::string start_sequence_;
  std::string before_initialization_sequence_;
  std::string after_initialization_sequence_;
  std::string end_sequence_;
  std::optional<double> route_timeout_sec_;
};

}  // namespace catchrobo2026_sequence

#endif  // CATCHROBO2026_SEQUENCE__SEQUENCE_CONFIG_HPP_
