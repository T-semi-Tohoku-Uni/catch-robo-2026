#include "catchrobo2026_sequence/sequence_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "nav_director/route_planner.hpp"

namespace catchrobo2026_sequence
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kWristTolerance = 1e-6;

geometry_msgs::msg::Pose route_pose(const Pose & point)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = point[0] / 1000.0;
  pose.position.y = point[1] / 1000.0;
  pose.position.z = point[2] / 1000.0;
  pose.orientation.z = std::sin(point[3] / 2.0);
  pose.orientation.w = std::cos(point[3] / 2.0);
  return pose;
}

nav_director::Point3D read_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  quaternion.normalize();
  const tf2::Matrix3x3 rotation(quaternion);
  return {pose.position.x * 1000.0, pose.position.y * 1000.0,
    pose.position.z * 1000.0, std::atan2(-rotation[0][1], rotation[1][1])};
}

bool legal_wrist(double value)
{
  return std::isfinite(value) && value >= -2.0 * kPi - kWristTolerance &&
         value <= kWristTolerance;
}

struct Route
{
  std::size_t first;
  std::size_t last;
  std::vector<Pose> targets;
};

Route collect_route(const std::vector<Step> & steps, std::size_t first)
{
  Route route{first, first, {}};
  for (;;) {
    if (route.last == steps.size() || steps[route.last].type != StepType::MOVE) {
      throw std::runtime_error("Waypoints require a following MOVE in the same group");
    }
    const auto & step = steps[route.last];
    route.targets.insert(route.targets.end(), step.waypoints.begin(), step.waypoints.end());
    route.targets.push_back(step.pose);
    if (!step.waypoint) {
      return route;
    }
    ++route.last;
  }
}

class Checker
{
public:
  SequenceCheckResult run(
    const std::vector<Step> & input_steps, const std::optional<CheckJoints> & initial,
    const SequenceCheckOptions & options)
  {
    if (initial && !set_joints(*initial, 0, "initial_state")) {
      return finish();
    }
    PreparedSequence prepared;
    try {
      auto pending = options.pending_waypoints;
      const auto initialize = std::find_if(input_steps.begin(), input_steps.end(),
        [](const Step & step) {return step.type == StepType::INITIALIZE;});
      if (options.clear_pending_waypoints || initialize != input_steps.end()) {
        result_.discarded_pending_waypoints = pending.size();
        pending.clear();
      }
      incoming_pending_ = pending.size();
      prepared = prepare_sequence(input_steps, pending);
    } catch (const std::exception & error) {
      fail(0, "planning_error", error.what());
      return finish();
    }
    const auto & steps = prepared.steps;
    for (std::size_t index = 0; index < steps.size(); ++index) {
      const auto & step = steps[index];
      try {
        if (step.type == StepType::INITIALIZE) {
          if (options.after_initialization_joints) {
            if (!set_joints(*options.after_initialization_joints, index, "initialization_state")) {
              break;
            }
          } else {
            joints_.reset();
            unknown(index, "initialization_state_unknown",
              "Initialization completion joints are unspecified; subsequent motion is unknown");
          }
        } else if (step.type == StepType::MOVE) {
          const auto route = collect_route(steps, index);
          if (joints_) {
            if (!check_route(route)) {
              break;
            }
            consumed_pending();
          } else {
            unknown(index, "initial_state_unknown",
              "Motion requires initial joint angles, including the unwrapped wrist angle");
          }
          index = route.last;
        } else if (step.type == StepType::SEQUENCE_START) {
          std::vector<Route> routes;
          std::size_t end = index + 1;
          for (; end < steps.size() && steps[end].type != StepType::SEQUENCE_END; ++end) {
            if (steps[end].type == StepType::SEQUENCE_START ||
              steps[end].type == StepType::INITIALIZE)
            {
              throw std::runtime_error("Sequence groups cannot nest or contain initialization");
            }
            if (steps[end].type == StepType::MOVE) {
              routes.push_back(collect_route(steps, end));
              end = routes.back().last;
            }
          }
          if (end == steps.size() || routes.empty()) {
            throw std::runtime_error("Sequence group requires a closing flag and at least one MOVE");
          }
          const auto intervals = collect_phi_travel_intervals(steps, index, end);
          if (joints_) {
            if (!check_group(step, index, routes, intervals)) {
              break;
            }
            consumed_pending();
          } else {
            unknown(index, "initial_state_unknown",
              "Sequence group requires initial joint angles, including the unwrapped wrist angle");
          }
          index = end;
        } else if (step.type == StepType::SEQUENCE_END ||
          step.type == StepType::PHI_TRAVEL_START || step.type == StepType::PHI_TRAVEL_END)
        {
          throw std::runtime_error("Group or phi interval flag has no corresponding sequence start");
        }
      } catch (const std::exception & error) {
        fail(index, "planning_error", error.what());
        break;
      }
    }
    if (result_.status == CheckStatus::FEASIBLE && !prepared.deferred_waypoints.empty()) {
      result_.pending_waypoints = std::move(prepared.deferred_waypoints);
      unknown(steps.size() < input_steps.size() ? steps.size() :
        steps.empty() ? 0 : steps.size() - 1, "pending_waypoints",
        "Waypoints are deferred until a later action supplies a MOVE; their route is not checked");
    }
    return finish();
  }

private:
  void consumed_pending()
  {
    result_.consumed_pending_waypoints += incoming_pending_;
    incoming_pending_ = 0;
  }

  bool set_joints(const CheckJoints & joints, std::size_t step, const std::string & code)
  {
    if (!std::all_of(joints.begin(), joints.end(), [](float value) {
        return std::isfinite(value);
      }) || !legal_wrist(joints[3]))
    {
      return fail(step, code,
        "Joint angles must be finite and the wrist must lie within [-2pi, 0]");
    }
    try {
      planner_.stateFromJoints(joints);
    } catch (const std::exception & error) {
      return fail(step, code, error.what());
    }
    joints_ = joints;
    return true;
  }

  bool check_route(const Route & route)
  {
    std::vector<nav_director::Point3D> targets;
    for (std::size_t i = 0; i < route.targets.size(); ++i) {
      const auto & target = route.targets[i];
      // Explicit waypoints travel through quaternions; the final target is scalar.
      targets.push_back(i + 1 == route.targets.size() ? nav_director::Point3D{
          target[0], target[1], target[2], target[3]} : read_pose(route_pose(target)));
    }
    const auto samples = planner_.generateRoute(planner_.stateFromJoints(*joints_), targets);
    if (samples.empty()) {
      return fail(route.first, "empty_route", "Route planner returned no samples");
    }
    const std::size_t route_index = result_.route_count++;
    for (std::size_t sample = 0; sample < samples.size(); ++sample) {
      const auto & point = samples[sample];
      CheckJoints next{};
      ++result_.sample_count;
      const auto command_pose = read_pose(route_pose({point.x, point.y, point.z, point.phi}));
      if (!planner_.solveIK(command_pose, next)) {
        std::ostringstream message;
        message << "Generated route sample is non-finite or outside the IK workspace: ["
                << command_pose.x << ", " << command_pose.y << ", " << command_pose.z
                << ", " << command_pose.phi << "] (mm, rad)";
        return fail(route.first, "unreachable_sample",
          message.str(), route_index, sample);
      }
      // Match the float arithmetic used by Joy before publishing normal commands.
      const float full_turn = 2.0f * static_cast<float>(kPi);
      if (next[3] > 0.0f || next[3] < -full_turn) {
        next[3] = std::fmod(next[3], full_turn);
        if (next[3] > 0.0f) {
          next[3] -= full_turn;
        }
      }
      observe(next, route.first, route_index, sample, true);
    }
    return true;
  }

  bool check_group(
    const Step & group, std::size_t index, const std::vector<Route> & routes,
    const std::vector<PhiTravelInterval> & intervals)
  {
    nav_director::PlanRotationGroup::Request request;
    request.allow_wrist_reversal = !group.rotation_group;
    request.limit_phi_travel = group.max_phi_travel.has_value();
    request.max_phi_travel = group.max_phi_travel.value_or(0.0);
    for (const auto & interval : intervals) {
      catchrobo2026_msgs::msg::PhiTravelInterval value;
      value.start_target = static_cast<uint32_t>(interval.start_target);
      value.end_target = static_cast<uint32_t>(interval.end_target);
      value.max_phi_travel = interval.max_phi_travel;
      request.phi_travel_intervals.push_back(value);
    }
    for (const auto & route : routes) {
      for (const auto & target : route.targets) {
        request.targets.push_back(route_pose(target));
      }
      request.route_ends.push_back(static_cast<uint32_t>(request.targets.size()));
    }
    const auto reply = planner_.planGroup(planner_.stateFromJoints(*joints_), request);
    if (!reply.success) {
      return fail(index, "sequence_group_infeasible", reply.message);
    }
    if (reply.routes.size() != routes.size() || !std::isfinite(reply.phi_travel) ||
      reply.phi_travel < 0.0 || reply.interval_phi_travel.size() != intervals.size())
    {
      return fail(index, "invalid_group_plan", "Sequence group planner returned invalid routes");
    }
    for (std::size_t i = 0; i < intervals.size(); ++i) {
      const double amount = reply.interval_phi_travel[i];
      if (!std::isfinite(amount) || amount < 0.0 || amount > intervals[i].max_phi_travel + 1e-5) {
        return fail(index, "invalid_group_plan", "Sequence group exceeded a phi interval limit");
      }
      std::ostringstream message;
      message << "Phi travel interval (target boundaries " << intervals[i].start_target << ".."
              << intervals[i].end_target << "): " << amount << " rad / "
              << intervals[i].max_phi_travel << " rad limit";
      result_.diagnostics.push_back({"info", "phi_travel_interval", message.str(), index, 0, 0});
    }
    for (std::size_t i = 0; i < routes.size(); ++i) {
      const auto & route = reply.routes[i];
      if (route.path.poses.empty() || route.wrist_angles.size() != route.path.poses.size() ||
        (!route.phi_angles.empty() && route.phi_angles.size() != route.path.poses.size()))
      {
        return fail(index, "invalid_group_plan", "Sequence group sample arrays do not match");
      }
      const std::size_t route_index = result_.route_count++;
      for (std::size_t sample = 0; sample < route.path.poses.size(); ++sample) {
        CheckJoints next{};
        auto point = read_pose(route.path.poses[sample].pose);
        ++result_.sample_count;
        if (!route.phi_angles.empty()) {
          point.phi = route.phi_angles[sample];
        }
        if (!planner_.solveIK(point, next) || !legal_wrist(route.wrist_angles[sample])) {
          return fail(index, "invalid_group_sample",
            "Sequence group contains an unreachable pose or invalid wrist command",
            route_index, sample);
        }
        next[3] = static_cast<float>(route.wrist_angles[sample]);
        observe(next, routes[i].first, route_index, sample, false);
      }
    }
    // This includes segment-interior extrema inspected by the shared planner.
    result_.phi_travel += reply.phi_travel;
    return true;
  }

  void observe(
    const CheckJoints & next, std::size_t step, std::size_t route, std::size_t sample,
    bool accumulate_phi)
  {
    const double amount = std::abs(static_cast<double>(next[3]) - (*joints_)[3]);
    result_.max_wrist_step = std::max(result_.max_wrist_step, amount);
    if (amount > kPi) {
      std::ostringstream message;
      message << "Wrist command changes by " << amount << " rad ("
              << amount * 180.0 / kPi << " deg) between checked samples";
      result_.diagnostics.push_back({"warning", "wrist_wrap_jump", message.str(),
        step, route, sample});
    }
    if (accumulate_phi) {
      const double before = static_cast<double>((*joints_)[0]) + (*joints_)[3];
      const double after = static_cast<double>(next[0]) + next[3];
      result_.phi_travel += std::abs(after - before);
    }
    joints_ = next;
  }

  bool fail(
    std::size_t step, const std::string & code, const std::string & message,
    std::size_t route = 0, std::size_t sample = 0)
  {
    result_.status = CheckStatus::INFEASIBLE;
    result_.diagnostics.push_back({"error", code, message, step, route, sample});
    joints_.reset();
    return false;
  }

  void unknown(std::size_t step, const std::string & code, const std::string & message)
  {
    result_.status = CheckStatus::UNKNOWN;
    if (result_.diagnostics.empty() || result_.diagnostics.back().code != code) {
      result_.diagnostics.push_back({"unknown", code, message, step, 0, 0});
    }
  }

  SequenceCheckResult finish()
  {
    result_.final_joints = joints_;
    switch (result_.status) {
      case CheckStatus::FEASIBLE:
        result_.message = "All generated routes satisfy the checked kinematics and constraints";
        break;
      case CheckStatus::INFEASIBLE:
        result_.message = "A route or initial state failed the offline checks";
        break;
      case CheckStatus::UNKNOWN:
        result_.message = result_.pending_waypoints.empty() ?
          "Motion feasibility is unknown because joint state is unspecified" :
          "Pending waypoints require a subsequent MOVE before feasibility can be determined";
        break;
    }
    return std::move(result_);
  }

  nav_director::RoutePlanner planner_;
  SequenceCheckResult result_;
  std::optional<CheckJoints> joints_;
  std::size_t incoming_pending_{0};
};

}  // namespace

SequenceCheckResult check_sequence_steps(
  const std::vector<Step> & steps, const std::optional<CheckJoints> & initial_joints,
  const SequenceCheckOptions & options)
{
  return Checker().run(steps, initial_joints, options);
}

}  // namespace catchrobo2026_sequence
