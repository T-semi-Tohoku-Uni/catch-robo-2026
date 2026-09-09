#include "catchrobo2026_sequence/sequence_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
    config.poses_ = poses;
    if (root["route_timeout_sec"]) {
      const double timeout = numeric(root["route_timeout_sec"], "route_timeout_sec");
      if (timeout <= 0.0 || timeout > MAX_DURATION_SEC) {
        fail("route_timeout_sec", "timeout must be in (0, 86400] seconds");
      }
      config.route_timeout_sec_ = timeout;
    }
    std::set<std::string> visiting_sequences;
    std::function<const std::vector<RawStep> &(const std::string &)> resolve_sequence;
    auto push = [&](std::vector<RawStep> & result, RawStep raw, const std::string & where) {
        if (expanded_size(result) + 1 + raw.waypoints.size() > kMaxExpandedSteps) {
          fail(where, "expanded sequence exceeds 10000 steps including waypoints");
        }
        result.push_back(std::move(raw));
      };
    auto append = [&](
        std::vector<RawStep> & result, const std::vector<RawStep> & included,
        const std::string & where)
      {
        if (expanded_size(included) > kMaxExpandedSteps - expanded_size(result)) {
          fail(where, "expanded sequence exceeds 10000 steps including waypoints");
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
          keys(step, at, {"move", "waypoint", "call", "pump", "endeffector", "wait",
            "rotation_group", "sequence_group", "phi_travel", "suction_check",
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
            if (count > (kMaxExpandedSteps - expanded_size(result)) / expanded_size(body)) {
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
          } else if (step["sequence_group"]) {
            const auto group = step["sequence_group"];
            const auto group_at = at + ".sequence_group";
            if (group.IsMap()) {
              keys(group, group_at, {"start", "rotation_group", "max_phi_travel"});
              if (scalar(group["start"], group_at + ".start") != "true") {
                fail(group_at + ".start", "expected true");
              }
              raw.step.type = StepType::SEQUENCE_START;
              if (group["rotation_group"]) {
                const auto value = scalar(
                  group["rotation_group"], group_at + ".rotation_group");
                if (value != "true" && value != "false") {
                  fail(group_at + ".rotation_group", "expected true or false");
                }
                raw.step.rotation_group = value == "true";
              }
              if (group["max_phi_travel"]) {
                const double limit = numeric(
                  group["max_phi_travel"], group_at + ".max_phi_travel");
                if (limit < 0.0) {
                  fail(group_at + ".max_phi_travel", "expected a nonnegative limit in radians");
                }
                raw.step.max_phi_travel = limit;
              }
            } else {
              const auto value = scalar(group, group_at);
              if (value != "start" && value != "end") {
                fail(group_at, "expected start, end or a start options mapping");
              }
              raw.step.type = value == "start" ?
                StepType::SEQUENCE_START : StepType::SEQUENCE_END;
            }
          } else if (step["phi_travel"]) {
            const auto interval = step["phi_travel"];
            const auto interval_at = at + ".phi_travel";
            if (interval.IsMap()) {
              keys(interval, interval_at, {"start", "limit"});
              if (scalar(interval["start"], interval_at + ".start") != "true") {
                fail(interval_at, "expected start: true and a limit in radians");
              }
              const double limit = numeric(interval["limit"], interval_at + ".limit");
              if (limit < 0.0) {
                fail(interval_at, "expected a nonnegative limit in radians");
              }
              raw.step.type = StepType::PHI_TRAVEL_START;
              raw.step.max_phi_travel = limit;
            } else {
              if (scalar(interval, interval_at) != "end") {
                fail(interval_at, "expected end or {start: true, limit: radians}");
              }
              raw.step.type = StepType::PHI_TRAVEL_END;
            }
          } else if (step["rotation_group"]) {
            const auto value = scalar(step["rotation_group"], at + ".rotation_group");
            if (value != "start" && value != "end") {
              fail(at + ".rotation_group", "expected start or end");
            }
            raw.step.type = value == "start" ?
              StepType::SEQUENCE_START : StepType::SEQUENCE_END;
            raw.step.rotation_group = value == "start";
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
  return compile_sequence(start_sequence_, "start_sequence", false);
}

std::vector<Step> SequenceConfig::compile_initialization() const
{
  auto before = before_initialization_sequence_.empty() ? std::vector<Step>{} :
    compile_sequence(before_initialization_sequence_, "before_initialization_sequence", false);
  // Homing changes the pose; each hook needs its own absolute anchor.
  auto after = after_initialization_sequence_.empty() ? std::vector<Step>{} :
    compile_sequence(after_initialization_sequence_, "after_initialization_sequence", false);
  if (expanded_size(before) + 1 + expanded_size(after) > kMaxExpandedSteps) {
    fail("initialization_sequence", "expanded sequence exceeds 10000 steps including waypoints");
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
  return compile_sequence(end_sequence_, "end_sequence", false);
}

std::vector<SequenceBinding> SequenceConfig::configured_bindings(const std::string & team) const
{
  if (team != "red" && team != "blue" && team != "both") {
    fail("team", "expected red, blue or both");
  }
  std::vector<SequenceBinding> result;
  for (const auto & [key, sequence] : bindings_) {
    const auto & [binding_team, kind, first, second] = key;
    if (!sequence.empty() && (team == "both" || team == binding_team)) {
      result.push_back({binding_team, kind, first, second, sequence});
    }
  }
  return result;
}

std::vector<Step> SequenceConfig::compile_named(const std::string & name) const
{
  if (sequences_.find(name) == sequences_.end()) {
    fail("sequences." + name, "unknown sequence");
  }
  return compile_sequence(name, "sequences." + name);
}

Pose SequenceConfig::named_pose(const std::string & name) const
{
  const auto found = poses_.find(name);
  if (found == poses_.end()) {
    fail("poses." + name, "unknown pose reference");
  }
  return found->second;
}

std::vector<Step> SequenceConfig::compile_sequence(
  const std::string & name, const std::string & where, bool allow_deferred_waypoints) const
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
      const Anchor * anchor = nullptr;
      if (raw.relative || std::any_of(raw.waypoints.begin(), raw.waypoints.end(),
          [](const RawPose & point) {return point.relative;}))
      {
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
      }
      // Inline points use the preceding anchor without changing the MOVE target's basis.
      for (const auto & point : raw.waypoints) {
        Pose resolved = point.pose;
        if (point.relative) {
          for (std::size_t i = 0; i < resolved.size(); ++i) {
            resolved[i] += anchor->value[i];
            if (!std::isfinite(resolved[i])) {
              fail(where, "relative waypoint overflows a finite coordinate");
            }
          }
        }
        step.waypoints.push_back(resolved);
      }
      if (raw.relative) {
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
  bool in_sequence_group = false;
  bool sequence_group_has_move = false;
  std::size_t group_start = 0;
  std::vector<std::optional<std::size_t>> group_at(result.size() + 1);
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto at = where + ".steps[" + std::to_string(i) + "]";
    if (in_sequence_group) {
      group_at[i] = group_start;
      if (result[i].type == StepType::IF_SUCTION || result[i].type == StepType::JUMP) {
        fail(at, "conditions and break cannot run inside a preplanned sequence group");
      }
    }
    const bool deferred_tail = i + 1 == result.size() &&
      allow_deferred_waypoints && !in_sequence_group;
    if (result[i].waypoint && !deferred_tail &&
      (i + 1 == result.size() || result[i + 1].type != StepType::MOVE))
    {
      fail(at,
        "waypoint movement must be followed immediately by another movement");
    }
    if (result[i].type == StepType::SEQUENCE_START) {
      if (in_sequence_group) {
        fail(at, "sequence groups cannot be nested");
      }
      in_sequence_group = true;
      group_start = i;
      sequence_group_has_move = false;
    } else if (result[i].type == StepType::SEQUENCE_END) {
      if (!in_sequence_group) {
        fail(at, "sequence_group end needs a preceding start");
      }
      if (!sequence_group_has_move) {
        fail(at, "sequence group must contain at least one movement");
      }
      collect_phi_travel_intervals(result, group_start, i);
      in_sequence_group = false;
    } else if ((result[i].type == StepType::PHI_TRAVEL_START ||
      result[i].type == StepType::PHI_TRAVEL_END) && !in_sequence_group)
    {
      fail(at, "phi_travel must be inside a sequence_group");
    } else if (result[i].type == StepType::INITIALIZE && in_sequence_group) {
      fail(at, "initialization cannot run inside a sequence group");
    } else if (result[i].type == StepType::MOVE && in_sequence_group) {
      sequence_group_has_move = true;
    }
  }
  if (in_sequence_group) {
    fail(where, "sequence_group start needs a matching end in the same action or hook");
  }
  std::size_t tail_start = result.size();
  while (tail_start > 0 && result[tail_start - 1].type == StepType::MOVE &&
    result[tail_start - 1].waypoint)
  {
    --tail_start;
  }
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto & step = result[i];
    if (step.type != StepType::IF_SUCTION && step.type != StepType::JUMP) {
      continue;
    }
    const auto at = where + ".steps[" + std::to_string(i) + "]";
    if (group_at[step.jump_index]) {
      fail(at, "a jump cannot enter an active sequence group");
    }
    if (tail_start < result.size() && step.jump_index > tail_start) {
      fail(at, "a jump cannot skip deferred waypoints; put them after the conditional");
    }
    if (step.jump_index > 0 && result[step.jump_index - 1].waypoint) {
      fail(at, "a jump cannot enter the middle of a waypoint route");
    }
  }
  return result;
}

std::vector<PhiTravelInterval> collect_phi_travel_intervals(
  const std::vector<Step> & steps, std::size_t group_start, std::size_t group_end)
{
  std::vector<PhiTravelInterval> result;
  std::optional<PhiTravelInterval> active;
  std::size_t target_count = 0;
  bool pending_waypoint = false;
  for (std::size_t i = group_start + 1; i < group_end; ++i) {
    const auto & step = steps.at(i);
    if (step.type == StepType::MOVE) {
      target_count += step.waypoints.size() + 1;
      pending_waypoint = step.waypoint;
    } else if (step.type == StepType::PHI_TRAVEL_START) {
      if (active || pending_waypoint || !step.max_phi_travel ||
        !std::isfinite(*step.max_phi_travel) || *step.max_phi_travel < 0.0)
      {
        fail("phi_travel", "invalid limit, nested interval or split waypoint route");
      }
      active = PhiTravelInterval{target_count, target_count, *step.max_phi_travel};
    } else if (step.type == StepType::PHI_TRAVEL_END) {
      if (!active || pending_waypoint || target_count == active->start_target) {
        fail("phi_travel", "end needs a start and a complete MOVE in the same interval");
      }
      active->end_target = target_count;
      result.push_back(*active);
      active.reset();
    }
  }
  if (active) {
    fail("phi_travel", "start needs a matching end in the same sequence_group");
  }
  return result;
}

PreparedSequence prepare_sequence(std::vector<Step> steps, const std::vector<Pose> & pending)
{
  if (pending.size() > kMaxExpandedSteps ||
    expanded_size(steps) > kMaxExpandedSteps - pending.size())
  {
    fail("continuation", "expanded sequence including pending waypoints exceeds 10000 points");
  }
  for (const auto & point : pending) {
    if (!std::all_of(point.begin(), point.end(), [](double value) {return std::isfinite(value);})) {
      fail("continuation", "pending waypoint must contain finite coordinates");
    }
  }
  PreparedSequence result;
  std::size_t end = steps.size();
  while (end > 0 && steps[end - 1].type == StepType::MOVE && steps[end - 1].waypoint) {
    --end;
  }
  for (std::size_t i = end; i < steps.size(); ++i) {
    result.deferred_waypoints.insert(result.deferred_waypoints.end(),
      steps[i].waypoints.begin(), steps[i].waypoints.end());
    result.deferred_waypoints.push_back(steps[i].pose);
  }
  steps.resize(end);
  for (auto & step : steps) {
    if (step.type == StepType::IF_SUCTION || step.type == StepType::JUMP) {
      // Compiled control flow can enter the deferred tail only at its start.
      if (step.jump_index > end) {
        fail("continuation", "a jump cannot skip deferred waypoints");
      }
    }
  }
  const auto first_move = std::find_if(steps.begin(), steps.end(), [](const Step & step) {
      return step.type == StepType::MOVE;
    });
  if (first_move == steps.end()) {
    result.deferred_waypoints.insert(result.deferred_waypoints.begin(), pending.begin(), pending.end());
  } else {
    if (!pending.empty()) {
      const auto first_index = static_cast<std::size_t>(first_move - steps.begin());
      for (std::size_t i = 0; i < first_index; ++i) {
        const auto & step = steps[i];
        if ((step.type == StepType::IF_SUCTION || step.type == StepType::JUMP) &&
          step.jump_index > first_index)
        {
          fail("continuation", "pending waypoints require an unconditional first MOVE");
        }
      }
    }
    first_move->waypoints.insert(first_move->waypoints.begin(), pending.begin(), pending.end());
    result.consumed_pending_waypoints = pending.size();
  }
  result.steps = std::move(steps);
  return result;
}

}  // namespace catchrobo2026_sequence
