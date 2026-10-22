#include "path_planner/trajectory_node.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace path_planner
{
namespace
{
double nowSeconds()
{
  using Clock = std::chrono::system_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

JointTrajectoryMsg toGeometricTrajectoryMsg(
  const Path & path,
  const std::vector<std::string> & joint_names,
  const std::string & request_id,
  double timestamp_sec)
{
  JointTrajectoryMsg msg;
  msg.request_id = request_id;
  msg.generated_timestamp_sec = timestamp_sec;
  msg.joint_names = joint_names;
  msg.points.reserve(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    JointTrajectoryPointMsg pt;
    pt.positions = path[i];
    pt.time_from_start_sec = 0.0;  // 未時間化経路（デバッグ用）
    msg.points.push_back(std::move(pt));
  }
  return msg;
}

JointTrajectoryMsg toTimedTrajectoryMsg(
  const TimedTrajectory & traj,
  const std::string & request_id,
  double timestamp_sec)
{
  JointTrajectoryMsg msg;
  msg.request_id = request_id;
  msg.generated_timestamp_sec = timestamp_sec;
  msg.joint_names = traj.joint_names;
  msg.points.reserve(traj.points.size());
  for (const auto & s : traj.points) {
    JointTrajectoryPointMsg pt;
    pt.positions = s.position;
    pt.velocities = s.velocity;
    pt.accelerations = s.acceleration;
    pt.effort = s.torque;
    pt.time_from_start_sec = s.time;
    msg.points.push_back(std::move(pt));
  }
  return msg;
}
}  // namespace

TrajectoryNode::TrajectoryNode(
  std::shared_ptr<TrajectoryPipeline> pipeline,
  ConcurrencyPolicy policy)
: pipeline_(std::move(pipeline)), policy_(policy)
{
  if (!pipeline_) {
    throw std::invalid_argument("TrajectoryNode requires a non-null TrajectoryPipeline");
  }
}

void TrajectoryNode::requestCancelCurrent()
{
  cancel_flag_.store(true);
}

bool TrajectoryNode::reorderJointState(
  const std::vector<std::string> & names,
  const State & values,
  State & out_ordered) const
{
  if (values.size() != kDof) {
    return false;
  }
  const auto & expected = pipeline_->config().joint_names;
  if (names.empty()) {
    out_ordered = values;
    return isFiniteState(out_ordered);
  }
  if (names.size() != kDof) {
    return false;
  }
  out_ordered.assign(kDof, 0.0);
  std::vector<bool> found(kDof, false);
  for (std::size_t i = 0; i < kDof; ++i) {
    for (std::size_t j = 0; j < kDof; ++j) {
      if (names[i] == expected[j]) {
        if (found[j]) {
          return false;  // 重複名
        }
        found[j] = true;
        out_ordered[j] = values[i];
        break;
      }
    }
  }
  for (bool f : found) {
    if (!f) {
      return false;
    }
  }
  return isFiniteState(out_ordered);
}

GenerateTrajectoryResponse TrajectoryNode::handleGenerateTrajectory(
  const GenerateTrajectoryRequest & request)
{
  const std::uint64_t seq = ++request_counter_;
  const std::string req_id = "req-" + std::to_string(seq);
  const double ts = nowSeconds();

  GenerateTrajectoryResponse response;
  response.request_id = req_id;

  // 並行実行制御（仕様書 第13.2節）
  {
    std::unique_lock<std::mutex> lock(active_mutex_);
    if (planning_active_) {
      if (policy_ == ConcurrencyPolicy::kReject) {
        response.success = false;
        response.failure_code = static_cast<std::uint16_t>(FailureCode::kPlanningFailed);
        response.message = "another trajectory planning request is currently active (rejected)";
        return response;
      }
      // cancel_previous: 実行中の計画にキャンセルを要求して完了を待つ
      cancel_flag_.store(true);
      while (planning_active_) {
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lock.lock();
      }
    }
    planning_active_ = true;
    cancel_flag_.store(false);
  }

  struct ActiveGuard
  {
    std::mutex & m;
    bool & active;
    ~ActiveGuard()
    {
      std::lock_guard<std::mutex> l(m);
      active = false;
    }
  } guard{active_mutex_, planning_active_};

  State start;
  State goal;
  if (!reorderJointState(request.start_joint_names, request.start_positions, start) ||
    !reorderJointState(request.goal_joint_names, request.goal_positions, goal))
  {
    std::lock_guard<std::mutex> pub_lock(pub_mutex_);
    has_timed_trajectory_ = false;
    last_timed_trajectory_ = JointTrajectoryMsg{};
    response.success = false;
    response.failure_code = static_cast<std::uint16_t>(FailureCode::kInvalidInput);
    response.message = "invalid joint state names or values (must match [theta1..theta4])";
    return response;
  }

  if (request.has_payload) {
    if (!request.payload.isValid()) {
      std::lock_guard<std::mutex> pub_lock(pub_mutex_);
      has_timed_trajectory_ = false;
      last_timed_trajectory_ = JointTrajectoryMsg{};
      response.success = false;
      response.failure_code = static_cast<std::uint16_t>(FailureCode::kInvalidInput);
      response.message = "invalid payload in request";
      return response;
    }
    pipeline_->updatePayload(request.payload);
  }

  const std::uint32_t effective_seed =
    (request.seed == 0) ? pipeline_->config().base_seed : request.seed;
  PipelineResult result = pipeline_->execute(start, goal, effective_seed, &cancel_flag_);

  // 実行直前に現在姿勢と軌道始点との差を検査する（仕様書 第13.2節）
  if (result && request.has_current_joint_state) {
    std::string exec_reason;
    if (!pipeline_->isTrajectoryValidForExecution(result, request.current_positions, &exec_reason)) {
      result.failure = FailureCode::kJointLimitViolation;
      result.failure_stage = "start_state_check";
      result.message = exec_reason;
      result.trajectory.points.clear();
    }
  }

  DiagnosticArrayMsg diag;
  diag.request_id = req_id;
  diag.timestamp_sec = ts;
  diag.level = result ? DiagnosticArrayMsg::Level::kOk : DiagnosticArrayMsg::Level::kError;
  diag.message = result.message;
  diag.key_values["request_id"] = req_id;
  diag.key_values["seed"] = std::to_string(effective_seed);
  diag.key_values["planner"] = "RrtConnect";
  diag.key_values["operation_mode"] = toString(pipeline_->operationMode());
  diag.key_values["candidate_count"] = std::to_string(result.candidate_count);
  diag.key_values["selected_candidate"] = std::to_string(result.selected_candidate);
  diag.key_values["failure_code"] = toString(result.failure);
  diag.key_values["failure_stage"] = result.failure_stage;
  diag.key_values["bottleneck"] = result.bottleneck;
  diag.key_values["duration"] = std::to_string(result.duration);
  diag.key_values["theta1_peak_torque"] = std::to_string(result.theta1_peak_torque);
  diag.key_values["theta1_rms_torque"] = std::to_string(result.theta1_rms_torque);
  diag.key_values["minimum_torque_margin"] = std::to_string(result.minimum_torque_margin);
  diag.key_values["theta1_rms_margin"] = std::to_string(result.theta1_rms_margin);
  diag.key_values["dynamics_version"] = std::to_string(result.dynamics_version);
  diag.key_values["scene_version"] = std::to_string(result.scene_version);
  for (const auto & stage : result.stage_durations) {
    diag.key_values["time_" + stage.first] = std::to_string(stage.second);
  }

  {
    std::lock_guard<std::mutex> pub_lock(pub_mutex_);
    last_diagnostics_ = diag;
    last_pipeline_result_ = result;
    if (!result.geometric_path.empty()) {
      last_geometric_path_ = toGeometricTrajectoryMsg(
        result.geometric_path, pipeline_->config().joint_names, req_id, ts);
    }
    if (result) {
      last_timed_trajectory_ = toTimedTrajectoryMsg(result.trajectory, req_id, ts);
      has_timed_trajectory_ = true;
      ++timed_publish_count_;
      response.trajectory = last_timed_trajectory_;
    } else {
      // 失敗時に直前の成功軌道を再利用させない（仕様書 第13.1, 15節）。
      has_timed_trajectory_ = false;
      last_timed_trajectory_ = JointTrajectoryMsg{};
    }
  }

  response.success = static_cast<bool>(result);
  response.failure_code = static_cast<std::uint16_t>(result.failure);
  response.message = result.message;
  response.theta1_peak_torque = result.theta1_peak_torque;
  response.theta1_rms_torque = result.theta1_rms_torque;
  response.minimum_torque_margin = result.minimum_torque_margin;
  return response;
}

bool TrajectoryNode::hasPublishedTimedTrajectory() const
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  return has_timed_trajectory_;
}

JointTrajectoryMsg TrajectoryNode::lastTimedTrajectory() const
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  return last_timed_trajectory_;
}

JointTrajectoryMsg TrajectoryNode::lastGeometricPath() const
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  return last_geometric_path_;
}

DiagnosticArrayMsg TrajectoryNode::lastDiagnostics() const
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  return last_diagnostics_;
}

PipelineResult TrajectoryNode::lastPipelineResult() const
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  return last_pipeline_result_;
}

}  // namespace path_planner
