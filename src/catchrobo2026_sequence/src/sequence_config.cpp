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

template<typename StepT>
std::size_t expanded_size(const std::vector<StepT> & steps)
{
  std::size_t count = steps.size();
  for (const auto & step : steps) {
    count += step.waypoints.size();
  }
  return count;
}

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
      "end_sequence", "route_timeout_sec"});
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

    auto target_value = [&](const YAML::Node & node, const std::string & where) {
        if (static_cast<bool>(node["absolute"]) == static_cast<bool>(node["relative"])) {
          fail(where, "specify exactly one of absolute or relative");
        }
        RawPose target;
        target.relative = static_cast<bool>(node["relative"]);
        const YAML::Node value = target.relative ? node["relative"] : node["absolute"];
        target.pose = !target.relative && value.IsScalar() ?
          resolve_pose(scalar(value, where + ".absolute")) : pose_value(value, where, numeric);
        return target;
      };

    auto waypoint_value = [&](const YAML::Node & node, const std::string & where) {
        if (node.IsMap()) {
          keys(node, where, {"absolute", "relative"});
          return target_value(node, where);
        }
        RawPose target;
        target.pose = node.IsScalar() ? resolve_pose(scalar(node, where)) :
          pose_value(node, where, numeric);
        return target;
      };

    SequenceConfig config;
    if (root["route_timeout_sec"]) {
      const double timeout = numeric(root["route_timeout_sec"], "route_timeout_sec");
      if (timeout <= 0.0 || timeout > MAX_DURATION_SEC) {
        fail("route_timeout_sec", "timeout must be in (0, 86400] seconds");
      }
      config.route_timeout_sec_ = timeout;
    }
    std::set<std::string> visiting_sequences;
    std::function<const std::vector<RawStep> &(const std::string &)> resolve_sequence;
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
        std::size_t expanded_count = 0;
        auto append = [&](const std::string & reference) {
            const auto & included = resolve_sequence(reference);
            const auto count = expanded_size(included);
            if (expanded_count + count > kMaxExpandedSteps) {
              fail(where, "expanded sequence exceeds 10000 steps including waypoints");
            }
            result.insert(result.end(), included.begin(), included.end());
            expanded_count += count;
          };
        if (node.IsScalar()) {
          append(scalar(node, where));
        } else {
          keys(node, where, {"extends", "steps"});
          if (node["extends"]) {
            const YAML::Node parents = node["extends"];
            if (parents.IsSequence()) {
              if (parents.size() == 0) {
                fail(where + ".extends", "expected at least one parent");
              }
              for (const auto & parent : parents) {
                append(scalar(parent, where + ".extends"));
              }
            } else {
              append(scalar(parents, where + ".extends"));
            }
          }
          if (node["steps"]) {
            const YAML::Node steps = node["steps"];
            if (!steps.IsSequence()) {
              fail(where + ".steps", "expected a list");
            }
            for (std::size_t i = 0; i < steps.size(); ++i) {
              const YAML::Node step = steps[i];
              const std::string at = where + ".steps[" + std::to_string(i) + "]";
              keys(step, at, {
                "move", "waypoint", "call", "pump", "endeffector", "wait", "rotation_group"});
              if (step.size() != 1) {
                fail(at, "a step must contain exactly one operation");
              }
              if (step["call"]) {
                append(scalar(step["call"], at + ".call"));
                continue;
              }
              RawStep raw;
              if (step["move"] || step["waypoint"]) {
                const bool standalone = static_cast<bool>(step["waypoint"]);
                const auto key = standalone ? "waypoint" : "move";
                const auto move_at = at + "." + key;
                const YAML::Node move = step[key];
                keys(move, move_at, standalone ? std::set<std::string>{"absolute", "relative"} :
                  std::set<std::string>{"absolute", "relative", "waypoint", "waypoints"});
                const auto target = target_value(move, move_at);
                raw.step.type = StepType::MOVE;
                raw.step.waypoint = standalone;
                if (move["waypoint"]) {
                  const auto waypoint = scalar(move["waypoint"], move_at + ".waypoint");
                  if (waypoint != "true" && waypoint != "false") {
                    fail(move_at + ".waypoint", "expected true or false");
                  }
                  raw.step.waypoint = waypoint == "true";
                }
                raw.relative = target.relative;
                raw.step.pose = target.pose;
                if (move["waypoints"]) {
                  const auto points = move["waypoints"];
                  if (!points.IsSequence()) {
                    fail(move_at + ".waypoints", "expected a list");
                  }
                  if (points.size() >= kMaxExpandedSteps) {
                    fail(move_at + ".waypoints",
                      "expanded sequence exceeds 10000 steps including waypoints");
                  }
                  for (std::size_t j = 0; j < points.size(); ++j) {
                    raw.waypoints.push_back(waypoint_value(
                      points[j], move_at + ".waypoints[" + std::to_string(j) + "]"));
                  }
                }
              } else if (step["rotation_group"]) {
                const auto value = scalar(step["rotation_group"], at + ".rotation_group");
                if (value != "start" && value != "end") {
                  fail(at + ".rotation_group", "expected start or end");
                }
                raw.step.type = value == "start" ?
                  StepType::ROTATION_START : StepType::ROTATION_END;
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
              } else {
                raw.step.type = StepType::WAIT;
                raw.step.seconds = numeric(step["wait"], at + ".wait");
                if (raw.step.seconds < 0.0 || raw.step.seconds > MAX_DURATION_SEC) {
                  fail(at + ".wait", "wait must be within 0..86400 seconds");
                }
              }
              expanded_count += 1 + raw.waypoints.size();
              if (expanded_count > kMaxExpandedSteps) {
                fail(where, "expanded sequence exceeds 10000 steps including waypoints");
              }
              result.push_back(raw);
            }
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
  const auto after = after_initialization_sequence_.empty() ? std::vector<Step>{} :
    compile_sequence(after_initialization_sequence_, "after_initialization_sequence");
  if (expanded_size(before) + 1 + expanded_size(after) > kMaxExpandedSteps) {
    fail("initialization_sequence", "expanded sequence exceeds 10000 steps including waypoints");
  }
  Step initialization;
  initialization.type = StepType::INITIALIZE;
  before.push_back(initialization);
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
  Pose anchor{};
  bool has_anchor = false;
  std::vector<Step> result;
  result.reserve(source.size());
  for (const auto & raw : source) {
    Step step = raw.step;
    if (step.type == StepType::MOVE) {
      // Inline points use the preceding anchor without changing the MOVE target's basis.
      for (const auto & point : raw.waypoints) {
        Pose resolved = point.pose;
        if (point.relative) {
          if (!has_anchor) {
            fail(where, "relative waypoint needs a preceding absolute movement");
          }
          for (std::size_t i = 0; i < resolved.size(); ++i) {
            resolved[i] += anchor[i];
            if (!std::isfinite(resolved[i])) {
              fail(where, "relative waypoint overflows a finite coordinate");
            }
          }
        }
        step.waypoints.push_back(resolved);
      }
      if (raw.relative) {
        if (!has_anchor) {
          fail(where, "relative movement needs a preceding absolute movement");
        }
        for (std::size_t i = 0; i < step.pose.size(); ++i) {
          step.pose[i] += anchor[i];
          if (!std::isfinite(step.pose[i])) {
            fail(where, "relative movement overflows a finite coordinate");
          }
        }
      } else {
        anchor = step.pose;
        has_anchor = true;
      }
    }
    result.push_back(step);
  }
  bool in_rotation_group = false;
  bool rotation_group_has_move = false;
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto at = where + ".steps[" + std::to_string(i) + "]";
    if (result[i].waypoint &&
      (i + 1 == result.size() || result[i + 1].type != StepType::MOVE))
    {
      fail(at,
        "waypoint movement must be followed immediately by another movement");
    }
    if (result[i].type == StepType::ROTATION_START) {
      if (in_rotation_group) {
        fail(at, "rotation groups cannot be nested");
      }
      in_rotation_group = true;
      rotation_group_has_move = false;
    } else if (result[i].type == StepType::ROTATION_END) {
      if (!in_rotation_group) {
        fail(at, "rotation_group end needs a preceding start");
      }
      if (!rotation_group_has_move) {
        fail(at, "rotation group must contain at least one movement");
      }
      in_rotation_group = false;
    } else if (result[i].type == StepType::INITIALIZE && in_rotation_group) {
      fail(at, "initialization cannot run inside a rotation group");
    } else if (result[i].type == StepType::MOVE && in_rotation_group) {
      rotation_group_has_move = true;
    }
  }
  if (in_rotation_group) {
    fail(where, "rotation_group start needs a matching end in the same action or hook");
  }
  return result;
}

}  // namespace catchrobo2026_sequence
