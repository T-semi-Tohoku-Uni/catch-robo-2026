#include "catchrobo2026_sequence/sequence_config.hpp"

#include <yaml-cpp/yaml.h>

#include <charconv>
#include <cmath>
#include <fstream>
#include <functional>
#include <iterator>
#include <set>
#include <sstream>
#include <utility>

namespace catchrobo2026_sequence
{
namespace
{

constexpr std::size_t kMaxExpandedSteps = 10000;
constexpr std::size_t kMaxReferenceDepth = 64;

[[noreturn]] void fail(const std::string & where, const std::string & message)
{
  throw ConfigError(where + ": " + message);
}

std::string scalar(const YAML::Node & node, const std::string & where)
{
  if (!node.IsScalar() || node.Scalar().empty()) {
    fail(where, "expected a nonempty scalar");
  }
  return node.Scalar();
}

void mapping(const YAML::Node & node, const std::string & where)
{
  if (!node.IsMap()) {
    fail(where, "expected a mapping");
  }
  std::set<std::string> seen;
  for (const auto & entry : node) {
    const auto key = scalar(entry.first, where + " key");
    if (!seen.insert(key).second) {
      fail(where, "duplicate key '" + key + "'");
    }
  }
}

void keys(
  const YAML::Node & node, const std::string & where,
  const std::set<std::string> & allowed)
{
  mapping(node, where);
  for (const auto & entry : node) {
    const auto key = entry.first.Scalar();
    if (allowed.count(key) == 0) {
      fail(where, "unknown key '" + key + "'");
    }
  }
}

double number(const YAML::Node & node, const std::string & where)
{
  if (!node.IsScalar()) {
    fail(where, "expected a finite number");
  }
  double value;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception &) {
    fail(where, "expected a finite number");
  }
  if (!std::isfinite(value)) {
    fail(where, "expected a finite number");
  }
  return value;
}

Pose pose_value(
  const YAML::Node & node, const std::string & where,
  const std::function<double(const YAML::Node &, const std::string &)> & numeric)
{
  if (!node.IsSequence() || node.size() != 4) {
    fail(where, "expected [x, y, z, phi] with exactly four values");
  }
  Pose pose;
  for (std::size_t i = 0; i < pose.size(); ++i) {
    pose[i] = numeric(node[i], where + "[" + std::to_string(i) + "]");
  }
  return pose;
}

int integer(const std::string & text, const std::string & where)
{
  int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    fail(where, "expected integer indices separated by one comma");
  }
  return value;
}

void validate_selection(
  const std::string & team, const std::string & kind, int first, int second,
  const std::string & where)
{
  if (team != "red" && team != "blue") {
    fail(where, "team must be red or blue");
  }
  if (kind != "pick" && kind != "place") {
    fail(where, "kind must be pick or place");
  }
  if (first < 0 || first > 3 ||
    (kind == "pick" && (second < 1 || second > 4)) ||
    (kind == "place" && (second < 0 || second > 1)))
  {
    fail(where, "indices are outside the UI selection range");
  }
}

}  // namespace

SequenceConfig SequenceConfig::load(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw ConfigError("cannot open sequence config: " + path);
  }
  const std::string yaml{
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (input.bad()) {
    throw ConfigError("cannot read sequence config: " + path);
  }
  try {
    return from_yaml(yaml);
  } catch (const ConfigError & error) {
    throw ConfigError(path + ": " + error.what());
  }
}

SequenceConfig SequenceConfig::from_yaml(const std::string & yaml)
{
  try {
    const auto documents = YAML::LoadAll(yaml);
    if (documents.size() != 1) {
      fail("config", "expected exactly one YAML document");
    }
    const YAML::Node root = documents.front();
    keys(root, "config", {"version", "values", "poses", "sequences", "bindings",
      "start_sequence", "before_initialization_sequence", "after_initialization_sequence",
      "end_sequence"});
    if (scalar(root["version"], "version") != "1") {
      fail("version", "only version 1 is supported");
    }
    const YAML::Node pose_nodes = root["poses"];
    const YAML::Node sequence_nodes = root["sequences"];
    mapping(pose_nodes, "poses");
    mapping(sequence_nodes, "sequences");

    const YAML::Node value_nodes = root["values"] ? root["values"] :
      YAML::Node(YAML::NodeType::Map);
    mapping(value_nodes, "values");
    std::map<std::string, double> values;
    std::set<std::string> visiting_values;
    std::function<double(const std::string &)> resolve_value;
    std::function<double(const YAML::Node &, const std::string &)> numeric;
    resolve_value = [&](const std::string & name) -> double {
        const auto found = values.find(name);
        if (found != values.end()) {
          return found->second;
        }
        const std::string where = "values." + name;
        const YAML::Node node = value_nodes[name];
        if (!node) {
          fail(where, "unknown numeric value reference");
        }
        if (!visiting_values.insert(name).second) {
          fail(where, "cyclic numeric value reference");
        }
        if (visiting_values.size() > kMaxReferenceDepth) {
          fail(where, "numeric value reference depth exceeds 64");
        }
        const double result = numeric(node, where);
        visiting_values.erase(name);
        values.emplace(name, result);
        return result;
      };
    numeric = [&](const YAML::Node & node, const std::string & where) -> double {
        if (node.IsScalar() && !node.Scalar().empty() && node.Scalar().front() == '$') {
          if (node.Scalar().size() == 1) {
            fail(where, "expected a value name after '$'");
          }
          return resolve_value(node.Scalar().substr(1));
        }
        return number(node, where);
      };
    for (const auto & entry : value_nodes) {
      resolve_value(entry.first.Scalar());
    }

    std::map<std::string, Pose> poses;
    std::set<std::string> visiting_poses;
    std::function<Pose(const std::string &)> resolve_pose;
    resolve_pose = [&](const std::string & name) -> Pose {
        const auto found = poses.find(name);
        if (found != poses.end()) {
          return found->second;
        }
        const std::string where = "poses." + name;
        const YAML::Node node = pose_nodes[name];
        if (!node) {
          fail(where, "unknown pose reference");
        }
        if (!visiting_poses.insert(name).second) {
          fail(where, "cyclic pose reference");
        }
        if (visiting_poses.size() > kMaxReferenceDepth) {
          fail(where, "pose reference depth exceeds 64");
        }
        Pose result;
        if (node.IsScalar()) {
          result = resolve_pose(scalar(node, where));
        } else if (node.IsMap()) {
          keys(node, where, {"extends", "x", "y", "z", "phi"});
          result = resolve_pose(scalar(node["extends"], where + ".extends"));
          const std::array<std::string, 4> axes{"x", "y", "z", "phi"};
          for (std::size_t i = 0; i < axes.size(); ++i) {
            if (node[axes[i]]) {
              result[i] = numeric(node[axes[i]], where + "." + axes[i]);
            }
          }
        } else {
          result = pose_value(node, where, numeric);
        }
        visiting_poses.erase(name);
        poses.emplace(name, result);
        return result;
      };
    for (const auto & entry : pose_nodes) {
      resolve_pose(entry.first.Scalar());
    }

    SequenceConfig config;
    std::set<std::string> visiting_sequences;
    std::function<const std::vector<RawStep> &(const std::string &)> resolve_sequence;
    auto push = [&](std::vector<RawStep> & result, RawStep raw, const std::string & where) {
        if (result.size() == kMaxExpandedSteps) {
          fail(where, "expanded sequence exceeds 10000 steps");
        }
        result.push_back(std::move(raw));
      };
    auto append = [&](
        std::vector<RawStep> & result, const std::vector<RawStep> & included,
        const std::string & where)
      {
        if (included.size() > kMaxExpandedSteps - result.size()) {
          fail(where, "expanded sequence exceeds 10000 steps");
        }
        const auto offset = result.size();
        for (auto raw : included) {
          if (raw.step.type == StepType::IF_SUCTION || raw.step.type == StepType::JUMP) {
            raw.step.jump_index += offset;
          }
          result.push_back(std::move(raw));
        }
      };
    std::size_t nesting_depth = 0;
    std::function<void(
        const YAML::Node &, const std::string &, std::vector<RawStep> &, bool)> parse_steps;
    parse_steps = [&](
        const YAML::Node & steps, const std::string & where,
        std::vector<RawStep> & result, bool in_loop)
      {
        if (!steps.IsSequence()) {
          fail(where, "expected a list");
        }
        if (++nesting_depth > kMaxReferenceDepth) {
          fail(where, "step nesting depth exceeds 64");
        }
        for (std::size_t i = 0; i < steps.size(); ++i) {
          const YAML::Node step = steps[i];
          const std::string at = where + "[" + std::to_string(i) + "]";
          keys(step, at, {"move", "call", "pump", "endeffector", "wait", "suction_check",
            "if", "for", "break", "fail"});
          if (step.size() != 1) {
            fail(at, "a step must contain exactly one operation");
          }
          if (step["call"]) {
            append(result, resolve_sequence(scalar(step["call"], at + ".call")), at);
            continue;
          }
          if (step["for"]) {
            const YAML::Node loop = step["for"];
            keys(loop, at + ".for", {"max_iterations", "steps"});
            const double count_value = numeric(loop["max_iterations"], at + ".for.max_iterations");
            if (count_value < 1.0 || count_value > kMaxExpandedSteps ||
              std::floor(count_value) != count_value)
            {
              fail(at + ".for.max_iterations", "expected an integer within 1..10000");
            }
            std::vector<RawStep> body;
            parse_steps(loop["steps"], at + ".for.steps", body, true);
            if (body.empty()) {
              continue;
            }
            const auto count = static_cast<std::size_t>(count_value);
            if (count > (kMaxExpandedSteps - result.size()) / body.size()) {
              fail(at + ".for", "expanded sequence exceeds 10000 steps");
            }
            const auto loop_end = result.size() + count * body.size();
            for (std::size_t iteration = 0; iteration < count; ++iteration) {
              const auto offset = result.size();
              for (auto raw : body) {
                if (raw.loop_break) {
                  raw.step.jump_index = loop_end;
                  raw.loop_break = false;
                } else if (raw.step.type == StepType::IF_SUCTION ||
                  raw.step.type == StepType::JUMP)
                {
                  raw.step.jump_index += offset;
                }
                result.push_back(std::move(raw));
              }
            }
            continue;
          }
          RawStep raw;
          raw.where = at;
          if (step["if"]) {
            const YAML::Node branch = step["if"];
            keys(branch, at + ".if", {"condition", "then", "else"});
            const auto condition = scalar(branch["condition"], at + ".if.condition");
            if (condition != "suction_success" && condition != "suction_failure") {
              fail(at + ".if.condition", "expected suction_success or suction_failure");
            }
            raw.step.type = StepType::IF_SUCTION;
            raw.step.condition_success = condition == "suction_success";
            const auto branch_index = result.size();
            push(result, std::move(raw), at);
            parse_steps(branch["then"], at + ".if.then", result, in_loop);
            if (branch["else"]) {
              RawStep skip_else;
              skip_else.step.type = StepType::JUMP;
              skip_else.where = at;
              const auto skip_index = result.size();
              push(result, std::move(skip_else), at);
              result[branch_index].step.jump_index = result.size();
              parse_steps(branch["else"], at + ".if.else", result, in_loop);
              result[skip_index].step.jump_index = result.size();
            } else {
              result[branch_index].step.jump_index = result.size();
            }
            continue;
          }
          if (step["move"]) {
            const YAML::Node move = step["move"];
            keys(move, at + ".move", {"absolute", "relative"});
            if (move.size() != 1) {
              fail(at + ".move", "specify exactly one of absolute or relative");
            }
            raw.step.type = StepType::MOVE;
            raw.relative = static_cast<bool>(move["relative"]);
            const YAML::Node value = raw.relative ? move["relative"] : move["absolute"];
            raw.step.pose = !raw.relative && value.IsScalar() ?
              resolve_pose(scalar(value, at + ".move.absolute")) :
              pose_value(value, at + ".move", numeric);
          } else if (step["pump"]) {
            const auto value = scalar(step["pump"], at + ".pump");
            raw.step.type = StepType::PUMP;
            if (value == "release") {
              raw.step.command = -1;
            } else if (value == "off") {
              raw.step.command = 0;
            } else if (value == "suction") {
              raw.step.command = 1;
            } else {
              fail(at + ".pump", "expected release, off or suction");
            }
          } else if (step["endeffector"]) {
            const auto value = numeric(step["endeffector"], at + ".endeffector");
            if (value != 0.0 && value != 1.0) {
              fail(at + ".endeffector", "expected command 0 or 1");
            }
            raw.step.type = StepType::ENDEFFECTOR;
            raw.step.command = value == 1.0 ? 1 : 0;
          } else if (step["suction_check"]) {
            const YAML::Node check = step["suction_check"];
            if (check.IsScalar()) {
              if (scalar(check, at + ".suction_check") != "wait") {
                fail(at + ".suction_check", "expected wait or {start: {timeout: seconds}}");
              }
              raw.step.type = StepType::SUCTION_CHECK_WAIT;
            } else {
              keys(check, at + ".suction_check", {"start"});
              keys(check["start"], at + ".suction_check.start", {"timeout"});
              raw.step.type = StepType::SUCTION_CHECK_START;
              raw.step.seconds = numeric(
                check["start"]["timeout"], at + ".suction_check.start.timeout");
              if (raw.step.seconds <= 0.0 || raw.step.seconds > MAX_DURATION_SEC) {
                fail(
                  at + ".suction_check.start.timeout", "timeout must be within (0, 86400] seconds");
              }
            }
          } else if (step["break"]) {
            if (scalar(step["break"], at + ".break") != "true") {
              fail(at + ".break", "expected true");
            }
            if (!in_loop) {
              fail(at + ".break", "break needs an enclosing for in the same sequence");
            }
            raw.step.type = StepType::JUMP;
            raw.loop_break = true;
          } else if (step["fail"]) {
            raw.step.type = StepType::FAIL;
            raw.step.message = scalar(step["fail"], at + ".fail");
          } else {
            raw.step.type = StepType::WAIT;
            raw.step.seconds = numeric(step["wait"], at + ".wait");
            if (raw.step.seconds < 0.0 || raw.step.seconds > MAX_DURATION_SEC) {
              fail(at + ".wait", "wait must be within 0..86400 seconds");
            }
          }
          push(result, std::move(raw), at);
        }
        --nesting_depth;
      };
    resolve_sequence = [&](const std::string & name) -> const std::vector<RawStep> & {
        const auto found = config.sequences_.find(name);
        if (found != config.sequences_.end()) {
          return found->second;
        }
        const std::string where = "sequences." + name;
        const YAML::Node node = sequence_nodes[name];
        if (!node) {
          fail(where, "unknown sequence reference");
        }
        if (!visiting_sequences.insert(name).second) {
          fail(where, "cyclic sequence reference");
        }
        if (visiting_sequences.size() > kMaxReferenceDepth) {
          fail(where, "sequence reference depth exceeds 64");
        }
        std::vector<RawStep> result;
        auto append_reference = [&](const std::string & reference) {
            append(result, resolve_sequence(reference), where);
          };
        if (node.IsScalar()) {
          append_reference(scalar(node, where));
        } else {
          keys(node, where, {"extends", "steps"});
          if (node["extends"]) {
            const YAML::Node parents = node["extends"];
            if (parents.IsSequence()) {
              if (parents.size() == 0) {
                fail(where + ".extends", "expected at least one parent");
              }
              for (const auto & parent : parents) {
                append_reference(scalar(parent, where + ".extends"));
              }
            } else {
              append_reference(scalar(parents, where + ".extends"));
            }
          }
          if (node["steps"]) {
            parse_steps(node["steps"], where + ".steps", result, false);
          }
        }
        if (result.empty() && node.IsMap() && !node["steps"] && !node["extends"]) {
          fail(where, "sequence must declare steps or extends");
        }
        visiting_sequences.erase(name);
        return config.sequences_.emplace(name, std::move(result)).first->second;
      };
    for (const auto & entry : sequence_nodes) {
      resolve_sequence(entry.first.Scalar());
    }
    auto lifecycle_reference = [&](const std::string & key, std::string & target) {
        if (root[key] && !root[key].IsNull()) {
          target = scalar(root[key], key);
          resolve_sequence(target);
        }
      };
    lifecycle_reference("start_sequence", config.start_sequence_);
    lifecycle_reference("before_initialization_sequence", config.before_initialization_sequence_);
    lifecycle_reference("after_initialization_sequence", config.after_initialization_sequence_);
    lifecycle_reference("end_sequence", config.end_sequence_);

    const YAML::Node bindings = root["bindings"];
    keys(bindings, "bindings", {"red", "blue"});
    for (const std::string team : {"red", "blue"}) {
      const std::string team_at = "bindings." + team;
      const YAML::Node team_node = bindings[team];
      keys(team_node, team_at, {"pick", "place"});
      for (const std::string kind : {"pick", "place"}) {
        const std::string at = team_at + "." + kind;
        const YAML::Node entries = team_node[kind];
        mapping(entries, at);
        for (const auto & entry : entries) {
          const auto key = entry.first.Scalar();
          const auto comma = key.find(',');
          if (comma == std::string::npos) {
            fail(at + "." + key, "expected 'row,column' or 'box,box_column'");
          }
          const int first = integer(key.substr(0, comma), at + "." + key);
          const int second = integer(key.substr(comma + 1), at + "." + key);
          validate_selection(team, kind, first, second, at + "." + key);
          const std::string name = entry.second.IsNull() ? "" :
            scalar(entry.second, at + "." + key);
          if (!name.empty()) {
            resolve_sequence(name);
          }
          const BindingKey binding_key{team, kind, first, second};
          if (!config.bindings_.emplace(binding_key, name).second) {
            fail(at + "." + key, "duplicate numeric binding indices");
          }
        }
      }
    }
    // Validate every enabled entry before the first robot command.
    config.compile_start();
    config.compile_initialization();
    config.compile_end();
    for (const auto & entry : config.bindings_) {
      if (!entry.second.empty()) {
        config.compile(
          std::get<0>(entry.first), std::get<1>(entry.first),
          std::get<2>(entry.first), std::get<3>(entry.first));
      }
    }
    return config;
  } catch (const YAML::Exception & error) {
    throw ConfigError(std::string("invalid sequence YAML: ") + error.what());
  }
}

std::vector<Step> SequenceConfig::compile(
  const std::string & team, const std::string & kind, int index1, int index2) const
{
  const std::string where = "bindings." + team + "." + kind + "." +
    std::to_string(index1) + "," + std::to_string(index2);
  validate_selection(team, kind, index1, index2, where);
  const auto binding = bindings_.find(BindingKey{team, kind, index1, index2});
  if (binding == bindings_.end() || binding->second.empty()) {
    fail(where, "target is unconfigured (missing or null binding)");
  }
  auto result = compile_sequence(binding->second, where);
  if (result.empty()) {
    fail(where, "PICK/PLACE sequence must contain at least one step");
  }
  return result;
}

std::vector<Step> SequenceConfig::compile_start() const
{
  if (start_sequence_.empty()) {
    return {};
  }
  return compile_sequence(start_sequence_, "start_sequence");
}

std::vector<Step> SequenceConfig::compile_initialization() const
{
  auto before = before_initialization_sequence_.empty() ? std::vector<Step>{} :
    compile_sequence(before_initialization_sequence_, "before_initialization_sequence");
  // Homing changes the pose; each hook needs its own absolute anchor.
  auto after = after_initialization_sequence_.empty() ? std::vector<Step>{} :
    compile_sequence(after_initialization_sequence_, "after_initialization_sequence");
  if (before.size() + 1 + after.size() > kMaxExpandedSteps) {
    fail("initialization_sequence", "expanded sequence exceeds 10000 steps");
  }
  Step initialization;
  initialization.type = StepType::INITIALIZE;
  before.push_back(initialization);
  for (auto & step : after) {
    if (step.type == StepType::IF_SUCTION || step.type == StepType::JUMP) {
      step.jump_index += before.size();
    }
  }
  before.insert(before.end(), after.begin(), after.end());
  return before;
}

std::vector<Step> SequenceConfig::compile_end() const
{
  if (end_sequence_.empty()) {
    return {};
  }
  return compile_sequence(end_sequence_, "end_sequence");
}

std::vector<Step> SequenceConfig::compile_sequence(
  const std::string & name, const std::string & where) const
{
  const auto & source = sequences_.at(name);
  struct Anchor
  {
    bool reachable{false};
    bool has_value{false};
    bool ambiguous{false};
    Pose value{};
  };
  // Keep anchors correlated with the last check result across conditionals.
  constexpr std::size_t no_result = 0;
  constexpr std::size_t pending = 1;
  constexpr std::size_t success = 2;
  constexpr std::size_t failure = 3;
  using Flow = std::array<Anchor, 4>;
  auto merge_anchor = [](Anchor & target, const Anchor & incoming) {
      if (!incoming.reachable) {
        return;
      }
      if (!target.reachable) {
        target = incoming;
        return;
      }
      if (target.ambiguous || incoming.ambiguous ||
        target.has_value != incoming.has_value ||
        (target.has_value && target.value != incoming.value))
      {
        target.ambiguous = true;
        target.has_value = false;
      }
    };
  auto merge_flow = [&](Flow & target, const Flow & incoming) {
      for (std::size_t state = 0; state < target.size(); ++state) {
        merge_anchor(target[state], incoming[state]);
      }
    };
  std::vector<Flow> incoming(source.size() + 1);
  incoming.front()[no_result].reachable = true;
  std::vector<Step> result;
  result.reserve(source.size());
  for (const auto & raw : source) {
    result.push_back(raw.step);
  }
  // Unrolled loops only jump forward, so one pass covers every incoming path.
  for (std::size_t index = 0; index < source.size(); ++index) {
    const auto & raw = source[index];
    auto & step = result[index];
    const auto & flow = incoming[index];
    const auto at = where + " -> " + raw.where;
    if ((step.type == StepType::IF_SUCTION || step.type == StepType::JUMP) &&
      (step.jump_index <= index || step.jump_index > source.size()))
    {
      fail(at, "invalid forward jump target");
    }
    bool reachable = false;
    for (const auto & anchor : flow) {
      reachable = reachable || anchor.reachable;
    }
    if (!reachable) {
      continue;
    }
    Flow next = flow;
    if (step.type == StepType::MOVE) {
      if (raw.relative) {
        const Anchor * anchor = nullptr;
        for (const auto & candidate : flow) {
          if (!candidate.reachable) {
            continue;
          }
          if (!candidate.has_value || candidate.ambiguous) {
            fail(at, "relative movement needs one preceding absolute anchor on every path");
          }
          if (anchor && anchor->value != candidate.value) {
            fail(at, "relative movement has different branch anchors; add an absolute movement");
          }
          anchor = &candidate;
        }
        for (std::size_t i = 0; i < step.pose.size(); ++i) {
          step.pose[i] += anchor->value[i];
          if (!std::isfinite(step.pose[i])) {
            fail(at, "relative movement overflows a finite coordinate");
          }
        }
      } else {
        for (auto & anchor : next) {
          if (anchor.reachable) {
            anchor.has_value = true;
            anchor.ambiguous = false;
            anchor.value = step.pose;
          }
        }
      }
    } else if (step.type == StepType::SUCTION_CHECK_START) {
      if (flow[pending].reachable) {
        fail(at, "suction_check start needs a wait for the pending check first");
      }
      next = Flow{};
      for (const auto & anchor : flow) {
        merge_anchor(next[pending], anchor);
      }
    } else if (step.type == StepType::SUCTION_CHECK_WAIT) {
      if (flow[no_result].reachable || flow[success].reachable || flow[failure].reachable) {
        fail(at, "suction_check wait needs a pending start on every path");
      }
      next = Flow{};
      next[success] = flow[pending];
      next[failure] = flow[pending];
    } else if (step.type == StepType::IF_SUCTION) {
      if (flow[no_result].reachable || flow[pending].reachable) {
        fail(at, "suction condition needs a completed suction_check wait on every path");
      }
      for (const auto state : {success, failure}) {
        const auto target = (state == success) == step.condition_success ?
          index + 1 : step.jump_index;
        merge_anchor(incoming[target][state], flow[state]);
      }
      continue;
    } else if (step.type == StepType::JUMP) {
      merge_flow(incoming[step.jump_index], flow);
      continue;
    } else if (step.type == StepType::FAIL) {
      continue;
    }
    merge_flow(incoming[index + 1], next);
  }
  if (incoming.back()[pending].reachable) {
    fail(where, "suction_check start must be followed by wait before sequence completion");
  }
  return result;
}

}  // namespace catchrobo2026_sequence
