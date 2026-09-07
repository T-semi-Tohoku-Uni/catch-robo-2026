#ifndef CATCHROBO2026_SEQUENCE__SEQUENCE_CONFIG_HPP_
#define CATCHROBO2026_SEQUENCE__SEQUENCE_CONFIG_HPP_

#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace catchrobo2026_sequence
{

using Pose = std::array<double, 4>;
inline constexpr double MAX_DURATION_SEC = 86400.0;

enum class StepType { MOVE, PUMP, ENDEFFECTOR, WAIT, INITIALIZE };

struct Step
{
  StepType type{StepType::MOVE};
  Pose pose{};
  int command{0};
  double seconds{0.0};
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

private:
  struct RawStep
  {
    Step step;
    bool relative{false};
  };

  using BindingKey = std::tuple<std::string, std::string, int, int>;
  std::vector<Step> compile_sequence(
    const std::string & name, const std::string & where) const;

  std::map<std::string, std::vector<RawStep>> sequences_;
  std::map<BindingKey, std::string> bindings_;
  std::string start_sequence_;
  std::string before_initialization_sequence_;
  std::string after_initialization_sequence_;
  std::string end_sequence_;
};

}  // namespace catchrobo2026_sequence

#endif  // CATCHROBO2026_SEQUENCE__SEQUENCE_CONFIG_HPP_
