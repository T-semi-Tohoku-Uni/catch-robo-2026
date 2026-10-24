#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <sys/random.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "catchrobo2026_msgs/srv/arm_target_gate.hpp"
#include "canrpc_bridge/srv/call.hpp"
#include "canrpc_bridge/msg/bridge_status.hpp"
#include "arm_trajectory_owner.hpp"

using namespace std::chrono_literals;
using Call = canrpc_bridge::srv::Call;
using Gate = catchrobo2026_msgs::srv::ArmTargetGate;
using Targets = std_msgs::msg::Float32MultiArray;

class ArmTargetClient : public rclcpp::Node {
public:
  ArmTargetClient() : Node("arm_target_client") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto rpc_service = declare_parameter<std::string>("canrpc_service", "canrpc/call", descriptor);
    endpoint_ = declare_parameter<std::string>("arm_target_endpoint", "arm_target", descriptor);
    timeout_sec_ = declare_parameter<double>("rpc_timeout_sec", 0.15, descriptor);
    if (!std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0) {
      throw std::invalid_argument("rpc_timeout_sec must be finite and positive");
    }
    period_ms_ = declare_parameter<int>("target_period_ms", 10, descriptor);
    if (period_ms_ < 10 || period_ms_ > 60000) {
      throw std::invalid_argument("target_period_ms must be in 10..60000 (at most 100 Hz)");
    }
    bridge_status_timeout_sec_ = declare_parameter<double>("bridge_status_timeout_sec", 2.5, descriptor);
    if (!std::isfinite(bridge_status_timeout_sec_) || bridge_status_timeout_sec_ <= 1.0) {
      throw std::invalid_argument("bridge_status_timeout_sec must be finite and greater than 1 second");
    }
    rpc_ = create_client<Call>(rpc_service);
    subscribe_targets();
    initialize_trajectory_owner();
    if (getrandom(&gate_token_, sizeof(gate_token_), 0) != sizeof(gate_token_) || gate_token_ == 0) {
      throw std::runtime_error("target gate random token source failed");
    }
    const auto status_topic = declare_parameter<std::string>("canrpc_status_topic", "canrpc/status", descriptor);
    bridge_status_ = create_subscription<canrpc_bridge::msg::BridgeStatus>(status_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](canrpc_bridge::msg::BridgeStatus::ConstSharedPtr status) {
        const bool valid_session = status->session_id.size() == 32 &&
          std::all_of(status->session_id.begin(), status->session_id.end(),
            [](unsigned char c) { return std::isxdigit(c) != 0; });
        const bool changed = valid_session && !bridge_session_.empty() && bridge_session_ != status->session_id;
        const bool expired = bridge_status_received_ && Clock::now() - bridge_status_seen_ >=
          std::chrono::duration<double>(bridge_status_timeout_sec_);
        bridge_status_seen_ = Clock::now();
        bridge_status_received_ = true;
        bridge_status_reason_ = status->reason.substr(0, 64);
        bridge_status_ready_ = valid_session && status->state == canrpc_bridge::msg::BridgeStatus::READY &&
          status->commands_enabled && status->socket_open;
        if (valid_session) bridge_session_ = status->session_id;
        if (changed || expired || !bridge_status_ready_) {
          if (changed || expired || !bridge_status_inhibited_) inhibit_bridge(
            changed ? "CAN-RPC activation session changed; explicit initialization required" :
            expired ? "CAN-RPC status heartbeat gap exceeded; explicit initialization required" :
            "CAN-RPC bridge not ready: " + status->reason);
          bridge_status_inhibited_ = true;
        } else bridge_status_inhibited_ = false;
      });
    const auto gate_service = declare_parameter<std::string>("target_gate_service", "arm_target/gate", descriptor);
    gate_ = create_service<Gate>(gate_service,
      [this](const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<Gate::Request> request) {
        Gate::Response response;
        if (request->pause) {
          paused_ = true;
          if (route_token_) revoked_route_until_ = route_expires_;
          route_token_ = 0;
          ++owner_generation_;
          if (trajectory_core_) trajectory_core_->requestCancelCurrent();
          if (trajectory_goal_ || trajectory_reserved_ || ending_) begin_ending(false, "explicit initialization pause", true);
          latest_.reset();
          if (pause_header_) {
            response.token = gate_token_;
            response.message = "target pause already pending";
            gate_->send_response(*header, response);
            return;
          }
          if (++gate_token_ == 0) ++gate_token_;
          pause_header_ = header;
          pause_deadline_ns_ = request->deadline_steady_ns;
          finish_pause();
          return;
        }
        response.token = gate_token_;
        if (request->token != gate_token_ || request->deadline_steady_ns <= steady_ns() ||
            !paused_ || pending_ || pause_header_ || planning_worker_.joinable() || !bridge_available()) {
          response.message = !bridge_available() ? bridge_unavailable_reason() :
            "target resume rejected: stale token/deadline or pause incomplete";
          gate_->send_response(*header, response);
          return;
        }
        if (resume_header_ || !rebase_->service_is_ready()) {
          response.message = "Joy rebase unavailable or already pending";
          gate_->send_response(*header, response);
          return;
        }
        resume_header_ = header;
        resume_deadline_ns_ = request->deadline_steady_ns;
        const auto resume_token = gate_token_;
        const auto resume_generation = ++resume_generation_;
        rebase_->async_send_request(std::make_shared<Trigger::Request>(),
          [this, resume_token, resume_generation](rclcpp::Client<Trigger>::SharedFuture future) {
            if (!resume_header_ || resume_generation != resume_generation_) return;
            Gate::Response result;
            result.token = gate_token_;
            try { result.success = future.get()->success && gate_token_ == resume_token &&
                steady_ns() < resume_deadline_ns_ && paused_ && !pending_ && !pause_header_ && bridge_available(); }
            catch (...) {}
            if (result.success) {
              latest_.reset(); faulted_ = false; bridge_status_inhibited_ = false;
              target_.reset(); subscribe_targets(); paused_ = false;
              result.message = "target resumed after Joy rebase; waiting for new input";
            } else { faulted_ = true; result.message = "Joy rebase failed or initialization deadline expired"; }
            gate_->send_response(*resume_header_, result); resume_header_.reset();
          });
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Arm target RPC adapter starts paused; explicit initialization is required");
  }
  ~ArmTargetClient() override {
    if (trajectory_core_) trajectory_core_->requestCancelCurrent();
    if (planning_worker_.joinable()) planning_worker_.join();
  }
private:
  using Clock = std::chrono::steady_clock;
  static uint64_t steady_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
  }
  bool bridge_available() const {
    return bridge_status_ready_ && Clock::now() - bridge_status_seen_ <
      std::chrono::duration<double>(bridge_status_timeout_sec_);
  }
  std::string bridge_unavailable_reason() const {
    if (!bridge_status_received_) return "target resume rejected: CAN-RPC status not received";
    if (!bridge_status_ready_) return "target resume rejected: CAN-RPC bridge not ready: " + bridge_status_reason_;
    return "target resume rejected: CAN-RPC status heartbeat expired";
  }
  void inhibit_bridge(const std::string &message) {
    paused_ = true;
    bridge_status_inhibited_ = true;
    if (route_token_) revoked_route_until_ = route_expires_;
    if (++gate_token_ == 0) ++gate_token_;
    ++resume_generation_;
    if (resume_header_) {
      Gate::Response response; response.token = gate_token_; response.message = message;
      gate_->send_response(*resume_header_, response); resume_header_.reset();
    }
    if (pending_) rpc_->remove_pending_request(request_id_);
    fault(message);
  }
  void subscribe_targets() {
    target_ = create_subscription<Targets>("target_joint_angles", rclcpp::QoS(1),
      [this](Targets::ConstSharedPtr message) {
        if (paused_ || faulted_ || trajectory_goal_ || trajectory_reserved_ || ending_ || message->data.size() != 3 || message->layout.data_offset != 0) return;
        const auto &dimensions = message->layout.dim;
        if (!dimensions.empty() && (dimensions.size() != 1 ||
            dimensions[0].size != 3 || dimensions[0].stride != 3)) return;
        std::array<float, 3> next;
        for (size_t axis = 0; axis < next.size(); ++axis) {
          if (!std::isfinite(message->data[axis])) return;
          next[axis] = message->data[axis];
        }
        latest_ = next;
      });
  }
  static uint32_t read_u32(const std::vector<uint8_t> &data, size_t offset) {
    return uint32_t(data[offset]) | uint32_t(data[offset + 1]) << 8 |
      uint32_t(data[offset + 2]) << 16 | uint32_t(data[offset + 3]) << 24;
  }
  static std::vector<uint8_t> encode(const std::array<float, 3> &values) {
    std::vector<uint8_t> data(12);
    for (size_t axis = 0; axis < values.size(); ++axis) {
      uint32_t bits;
      std::memcpy(&bits, &values[axis], sizeof(bits));
      for (size_t byte = 0; byte < 4; ++byte) data[axis * 4 + byte] = uint8_t(bits >> (byte * 8));
    }
    return data;
  }
  void finish_pause() {
    if (!pause_header_) return;
    const bool expired = steady_ns() >= pause_deadline_ns_;
    if ((pending_ || ending_ || planning_worker_.joinable() || Clock::now() < revoked_route_until_) && !expired) return;
    Gate::Response response;
    response.token = gate_token_;
    response.success = !expired;
    response.message = expired ? "target pause deadline exceeded; gate remains paused" :
      faulted_ ? "target gate paused; RPC outcome fault remains latched" :
      "target gate paused; RPC inflight drained";
    try { gate_->send_response(*pause_header_, response); }
    catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Target pause response failed: %s", error.what());
    }
    pause_header_.reset();
  }
  void fault(const std::string &message) {
    faulted_ = true;
    route_token_ = 0;
    if (trajectory_goal_ || trajectory_reserved_ || ending_) begin_ending(false, message, true);
    pending_ = false;
    latest_.reset();
    ++token_;
    RCLCPP_ERROR(get_logger(), "Arm target RPC fault latched: %s", message.c_str());
    finish_pause();
  }
  void tick() {
    const auto now = Clock::now();
    if (!bridge_available() && !bridge_status_inhibited_) {
      inhibit_bridge("CAN-RPC status missing or expired; explicit initialization required");
    }
    if (pending_ && now >= deadline_) {
      rpc_->remove_pending_request(request_id_);
      fault("response deadline exceeded; execution outcome unknown");
    }
    if (resume_header_ && steady_ns() >= resume_deadline_ns_) {
      Gate::Response result; result.token = gate_token_;
      result.message = "Joy rebase deadline expired; gate remains paused";
      gate_->send_response(*resume_header_, result); resume_header_.reset();
    }
    owner_tick();
    finish_pause();
    if (paused_ || faulted_ || !bridge_available() || pending_ || !latest_ || now - last_sent_ < std::chrono::milliseconds(period_ms_)) return;
    if (!rpc_->service_is_ready()) {
      fault("CAN-RPC service unavailable");
      return;
    }
    const auto data = encode(*latest_);
    latest_.reset();
    auto request = std::make_shared<Call::Request>();
    request->endpoint = endpoint_;
    request->data = data;
    pending_ = true;
    const auto token = ++token_;
    const auto trajectory_epoch = trajectory_epoch_;
    const bool trajectory_request = bool(trajectory_goal_);
    last_sent_ = now;
    deadline_ = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_sec_));
    try {
      request_id_ = rpc_->async_send_request(request,
        [this, token, data, trajectory_epoch, trajectory_request](rclcpp::Client<Call>::SharedFuture future) {
          if (!pending_ || token != token_) return;
          if (Clock::now() >= deadline_) {
            fault("late response; execution outcome unknown");
            return;
          }
          try {
            const auto reply = future.get();
            const bool confirmed = reply->outcome == 0 && reply->data.size() == 20 &&
              read_u32(reply->data, 0) == 0 && read_u32(reply->data, 16) == 7 &&
              std::equal(data.begin(), data.end(), reply->data.begin() + 4);
            if (!confirmed) { fault("native command not confirmed: " + reply->message); return; }
            pending_ = false;
            if (trajectory_goal_ && trajectory_request && trajectory_epoch == trajectory_epoch_ && data != encode(final_target_)) final_ack_ = false;
            if (trajectory_goal_ && trajectory_request && trajectory_epoch == trajectory_epoch_ && data == encode(final_target_)) {
              final_ack_ = true; final_ack_at_ = Clock::now();
            }
            finish_pause();
          } catch (const std::exception &error) { fault(error.what()); }
        }).request_id;
    } catch (const std::exception &error) { fault(error.what()); }
  }
  using Generate = catchrobo2026_msgs::srv::GenerateTrajectory;
  using Trajectory = catchrobo2026_msgs::action::FollowTrajectory;
  using TrajectoryHandle = rclcpp_action::ServerGoalHandle<Trajectory>;
  using Lease = catchrobo2026_msgs::srv::ArmCommandLease;
  using Trigger = std_srvs::srv::Trigger;
  static double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
  void initialize_trajectory_owner() {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    const auto dynamics = declare_parameter<std::string>("trajectory_dynamics_config", "", d);
    const auto pipeline = declare_parameter<std::string>("trajectory_pipeline_config", "", d);
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout_sec", 30.0, d);
    lease_ms_ = declare_parameter<int>("route_lease_duration_ms", 500, d);
    if (!std::isfinite(trajectory_timeout_) || trajectory_timeout_ <= 0 || lease_ms_ < 350 || lease_ms_ > 5000)
      throw std::invalid_argument("Invalid trajectory timeout or route lease duration");
    rebase_ = create_client<Trigger>("joy/rebase");
    if (!dynamics.empty() && !pipeline.empty()) {
      try {
        auto dyn = path_planner::loadRobotDynamicsConfig(path_planner::parseYamlFile(dynamics));
        auto cfg = path_planner::loadTrajectoryPipelineConfig(path_planner::parseYamlFile(pipeline), dyn.chain);
        cfg.active_axes = 3;
        cfg.joint_bounds.lower[3] = cfg.joint_bounds.upper[3] = 0;
        cfg.distance_weights[3] = 0;
        auto core_pipeline = std::make_shared<path_planner::TrajectoryPipeline>(cfg, dyn);
        if (!core_pipeline->isHardwareOutputAllowed()) throw std::runtime_error("measured hardware dynamics required");
        path_planner::TrajectoryFollowerConfig follower;
        follower.active_axes = 3;
        follower.execution_timeout_sec = trajectory_timeout_;
        trajectory_core_ = std::make_shared<path_planner::TrajectoryNode>(core_pipeline,
          path_planner::ConcurrencyPolicy::kReject, follower);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Timed trajectory disabled: %s", e.what());
      }
    }
    current_ = create_subscription<Targets>("current_joints", rclcpp::QoS(1), [this](Targets::ConstSharedPtr msg) {
      if (!trajectory_core_) return;
      std::vector<double> q(msg->data.begin(), msg->data.end());
      if (q.size() == 3 && msg->layout.data_offset == 0) q.push_back(0);
      else q.clear();
      trajectory_core_->follower()->updateCurrentJoints(q, seconds());
      feedback_at_owner_ = Clock::now();
    });
    generate_ = create_service<Generate>("generate_trajectory", [this](
      const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Generate::Request> request) {
      Generate::Response response;
      response.failure_code = 1;
      if (!trajectory_core_ || trajectory_goal_ || trajectory_reserved_ || route_token_ || ending_ || pending_ || planning_worker_.joinable()) {
        response.message = "Measured hardware dynamics unavailable or arm owner busy";
        generate_->send_response(*header, response); return;
      }
      const std::vector<std::string> names{"theta1", "theta2", "theta3"};
      if (request->start.position.size() != 3 || request->goal.position.size() != 3 || request->payload.has_payload ||
          (!request->start.name.empty() && request->start.name != names) ||
          (!request->goal.name.empty() && request->goal.name != names) ||
          !trajectory_core_->follower()->hasFreshJoints(seconds())) {
        response.message = "Fresh three-axis states in canonical order required; payload changes unsupported";
        generate_->send_response(*header, response); return;
      }
      path_planner::GenerateTrajectoryRequest input;
      input.start_positions = request->start.position; input.start_positions.push_back(0);
      input.goal_positions = request->goal.position; input.goal_positions.push_back(0);
      input.seed = request->seed; input.has_current_joint_state = true;
      input.current_positions = trajectory_core_->follower()->currentJoints();
      generate_header_ = header;
      generate_deadline_ = Clock::now() + std::chrono::seconds(5);
      start_worker([this, input]() {
        generated_result_ = trajectory_core_->handleGenerateTrajectory(input);
      });
    });
    trajectory_server_ = rclcpp_action::create_server<Trajectory>(this, "follow_trajectory",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Trajectory::Goal> goal) {
        if (!trajectory_core_ || !bridge_available() || paused_ || faulted_ || route_token_ || trajectory_goal_ ||
            trajectory_reserved_ || ending_ || planning_worker_.joinable() || !goal->start || goal->plan_from_start_goal || goal->trajectory.points.empty() ||
            goal->trajectory.points.size() > 10000 || goal->payload.has_payload ||
            !trajectory_core_->follower()->hasFreshJoints(seconds())) return rclcpp_action::GoalResponse::REJECT;
        path_planner::FollowTrajectoryGoalRequest request;
        request.trajectory.frame_id = goal->trajectory.header.frame_id;
        if (!goal->trajectory.joint_names.empty() && goal->trajectory.joint_names !=
            std::vector<std::string>{"theta1", "theta2", "theta3"}) return rclcpp_action::GoalResponse::REJECT;
        request.trajectory.joint_names = {"theta1", "theta2", "theta3", "theta4"};
        for (const auto &point : goal->trajectory.points) {
          if (point.positions.size() != 3 || (!point.velocities.empty() && point.velocities.size() != 3) ||
              (!point.accelerations.empty() && point.accelerations.size() != 3) || !point.effort.empty())
            return rclcpp_action::GoalResponse::REJECT;
          path_planner::JointTrajectoryPointMsg out;
          out.positions = point.positions; out.positions.push_back(0);
          out.velocities = point.velocities.empty() ? std::vector<double>(3, 0) : point.velocities;
          out.accelerations = point.accelerations.empty() ? std::vector<double>(3, 0) : point.accelerations;
          out.velocities.push_back(0); out.accelerations.push_back(0);
          out.time_from_start_sec = double(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9;
          request.trajectory.points.push_back(out);
        }
        planned_request_ = std::move(request);
        const auto &final = goal->trajectory.points.back().positions;
        final_target_ = {float(final[0]), float(final[1]), float(final[2])};
        ++trajectory_epoch_; final_ack_ = false; core_finished_ = false;
        maximum_tracking_error_ = std::numeric_limits<double>::quiet_NaN();
        latest_.reset(); trajectory_reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<TrajectoryHandle>) {
        begin_ending(false, "trajectory canceled", false);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<TrajectoryHandle> goal) {
        trajectory_reserved_ = false; trajectory_goal_ = goal;
        trajectory_started_ = Clock::now();
        validation_pending_ = true;
        validation_waiting_ = true;
      });
    lease_ = create_service<Lease>("arm_target/route_lease", [this](
      const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Lease::Request> request) {
      Lease::Response response;
      response.token = route_token_;
      if (request->deadline_steady_ns <= steady_ns()) response.message = "lease request expired";
      else if (request->operation == Lease::Request::ACQUIRE && bridge_available() && !paused_ && !faulted_ &&
          !trajectory_goal_ && !trajectory_reserved_ && !ending_ && !route_token_ && !planning_worker_.joinable()) {
        if (++gate_token_ == 0) ++gate_token_;
        route_token_ = gate_token_;
        route_expires_ = Clock::now() + std::chrono::milliseconds(lease_ms_);
        latest_.reset();
        response.success = true; response.token = route_token_;
      } else if (request->token && request->token == route_token_ && Clock::now() < route_expires_) {
        if (request->operation == Lease::Request::RENEW && bridge_available() && !paused_ && !faulted_) {
          route_expires_ = Clock::now() + std::chrono::milliseconds(lease_ms_); response.success = true;
        } else if (request->operation == Lease::Request::RELEASE && !ending_) {
          route_token_ = 0; latest_.reset();
          lease_release_header_ = header;
          begin_ending(true, "route released", false);
          return;
        }
      }
      if (response.success) response.expires_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        route_expires_.time_since_epoch()).count();
      else response.message = "route owner unavailable, stale token, or deadline";
      lease_->send_response(*header, response);
    });
    cancel_trajectory_ = create_service<Trigger>("arm_target/cancel_trajectory", [this](
      const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Trigger::Request>) {
      Trigger::Response response;
      if (cancel_header_) { response.message = "trajectory cancellation already pending"; cancel_trajectory_->send_response(*header, response); return; }
      if (!trajectory_goal_ && !trajectory_reserved_ && !ending_) {
        response.success = !faulted_; response.message = faulted_ ? "target fault latched" : "trajectory idle";
        cancel_trajectory_->send_response(*header, response); return;
      }
      cancel_header_ = header;
      begin_ending(false, "trajectory canceled by stop", false);
    });
  }
  void begin_ending(bool success, const std::string &message, bool latch) {
    ++owner_generation_;
    if (trajectory_core_) trajectory_core_->requestCancelCurrent();
    latest_.reset();
    if (!ending_) {
      ending_ = true; ending_success_ = success; ending_message_ = message;
      ending_deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(timeout_sec_ + 1.0));
    }
    if (latch) { paused_ = true; ending_success_ = false; ending_message_ = message; }
  }
  void finish_ending(bool restored) {
    if (!restored) { paused_ = true; faulted_ = true; ending_success_ = false; }
    latest_.reset();
    target_.reset(); subscribe_targets();
    if (trajectory_goal_) {
      auto result = std::make_shared<Trajectory::Result>();
      result->success = ending_success_ && restored;
      result->failure_code = result->success ? 0 : 1;
      result->message = ending_message_;
      result->final_position_error_mm = std::numeric_limits<double>::quiet_NaN();
      result->final_yaw_error_rad = std::numeric_limits<double>::quiet_NaN();
      result->max_joint_tracking_error_rad = maximum_tracking_error_;
      if (trajectory_core_ && trajectory_core_->follower()->hasFreshJoints(seconds())) {
        const auto q = trajectory_core_->follower()->currentJoints();
        const path_planner::State final{final_target_[0], final_target_[1], final_target_[2], 0};
        const auto actual = path_planner::forwardKinematicsPose(trajectory_core_->follower()->chain(), q);
        const auto desired = path_planner::forwardKinematicsPose(trajectory_core_->follower()->chain(), final);
        const double dx=actual.x_mm-desired.x_mm, dy=actual.y_mm-desired.y_mm, dz=actual.z_mm-desired.z_mm;
        result->final_position_error_mm = std::sqrt(dx*dx+dy*dy+dz*dz);
      }
      result->elapsed_time_sec = std::chrono::duration<double>(Clock::now() - trajectory_started_).count();
      if (trajectory_goal_->is_canceling()) trajectory_goal_->canceled(result);
      else if (result->success) trajectory_goal_->succeed(result);
      else trajectory_goal_->abort(result);
      trajectory_goal_.reset();
    }
    trajectory_reserved_ = false; validation_waiting_ = false; validation_pending_ = false;
    if (lease_release_header_) {
      Lease::Response response; response.success = restored && !faulted_;
      response.message = ending_message_;
      lease_->send_response(*lease_release_header_, response); lease_release_header_.reset();
    }
    if (cancel_header_) {
      Trigger::Response response; response.success = restored && !faulted_;
      response.message = ending_message_;
      cancel_trajectory_->send_response(*cancel_header_, response); cancel_header_.reset();
    }
    ending_ = false; rebase_pending_ = false;
  }
  void owner_tick() {
    const auto now = Clock::now();
    if (generate_header_ && now >= generate_deadline_) {
      ++owner_generation_;
      trajectory_core_->requestCancelCurrent();
      Generate::Response response;
      response.failure_code = 1; response.message = "Planning deadline exceeded; result discarded";
      generate_->send_response(*generate_header_, response); generate_header_.reset();
    }
    poll_worker();
    if (trajectory_goal_ && !ending_ && now - trajectory_started_ >= std::chrono::duration<double>(trajectory_timeout_))
      begin_ending(false, "trajectory execution deadline", true);
    if (validation_waiting_ && !pending_ && !ending_ && !paused_) {
      validation_waiting_ = false;
      const auto input = planned_request_;
      start_worker([this, input]() {
        validation_success_ = trajectory_core_->acceptFollowTrajectoryGoal(input, seconds(), &validation_failure_);
      });
    }
    if (route_token_ && now >= route_expires_) {
      route_token_ = 0; paused_ = true; faulted_ = true; latest_.reset();
      RCLCPP_ERROR(get_logger(), "Route lease expired; explicit initialization required");
    }
    if (trajectory_goal_ && !ending_ && !validation_pending_) {
      if (now - trajectory_started_ >= std::chrono::duration<double>(trajectory_timeout_))
        begin_ending(false, "trajectory execution deadline", true);
      else {
        const auto step = core_finished_ ? path_planner::FollowerStepOutput{} : trajectory_core_->stepFollowTrajectory(seconds());
        if (!core_finished_ && std::isfinite(step.max_joint_tracking_error_rad))
          maximum_tracking_error_ = step.max_joint_tracking_error_rad;
        if (step.finished && !step.success) begin_ending(false, step.message, true);
        else if (step.finished && step.success) core_finished_ = true;
        else if (step.target_joint_angles.size() == 4) {
          latest_ = std::array<float, 3>{float(step.target_joint_angles[0]), float(step.target_joint_angles[1]), float(step.target_joint_angles[2])};
          auto feedback = std::make_shared<Trajectory::Feedback>();
          feedback->progress_ratio = step.progress_ratio;
          feedback->time_remaining_sec = step.time_remaining_sec;
          feedback->distance_remaining_mm = step.distance_remaining_mm;
          for (size_t i = 0; i < 3; ++i) {
            feedback->desired_positions[i] = step.target_joint_angles[i];
            if (step.actual_joint_angles.size() == 4) feedback->actual_positions[i] = step.actual_joint_angles[i];
            if (step.joint_tracking_error.size() == 4) feedback->tracking_error[i] = step.joint_tracking_error[i];
          }
          trajectory_goal_->publish_feedback(feedback);
        }
      }
    }
    if (trajectory_goal_ && core_finished_ && !ending_) {
      if (!final_ack_) { if (!pending_) latest_ = final_target_; }
      else if (!pending_ && feedback_at_owner_ > final_ack_at_ &&
          trajectory_core_->follower()->hasFreshJoints(seconds())) {
        const auto q = trajectory_core_->follower()->currentJoints();
        bool reached = q.size() == 4;
        for (size_t i = 0; reached && i < 3; ++i)
          reached = std::abs(q[i] - final_target_[i]) <= (i == 0 ? 0.05 : 0.01);
        if (reached) {
          const path_planner::State target{final_target_[0], final_target_[1], final_target_[2], 0};
          const auto actual_pose = path_planner::forwardKinematicsPose(trajectory_core_->follower()->chain(), q);
          const auto final_pose = path_planner::forwardKinematicsPose(trajectory_core_->follower()->chain(), target);
          const double dx = actual_pose.x_mm-final_pose.x_mm, dy=actual_pose.y_mm-final_pose.y_mm, dz=actual_pose.z_mm-final_pose.z_mm;
          if (std::sqrt(dx*dx+dy*dy+dz*dz) <= 30.0) begin_ending(true, "final RPC applied and fresh three-axis arrival confirmed", false);
        }
      }
    }
    if (!ending_) return;
    if (now >= ending_deadline_) { finish_ending(false); return; }
    if (pending_ || planning_worker_.joinable()) return;
    if (paused_ || faulted_) { finish_ending(false); return; }
    if (!rebase_pending_) {
      if (!rebase_->service_is_ready()) { ending_message_ += "; Joy rebase unavailable"; finish_ending(false); return; }
      rebase_pending_ = true;
      const auto generation = ++rebase_generation_;
      rebase_->async_send_request(std::make_shared<Trigger::Request>(), [this, generation](rclcpp::Client<Trigger>::SharedFuture future) {
        if (!ending_ || generation != rebase_generation_) return;
        bool success = false;
        try { success = Clock::now() < ending_deadline_ && future.get()->success; } catch (...) {}
        if (!success) ending_message_ += "; Joy rebase not confirmed";
        finish_ending(success);
      });
    }
  }
  template<typename Function> void start_worker(Function function) {
    if (planning_worker_.joinable()) planning_worker_.join();
    planning_active_ = true;
    worker_generation_ = owner_generation_;
    planning_worker_ = std::thread([this, function]() {
      try { function(); }
      catch (const std::exception &error) {
        validation_success_ = false; validation_failure_.message = error.what();
        generated_result_.success = false; generated_result_.message = error.what();
      }
      planning_active_ = false;
    });
  }
  void poll_worker() {
    if (!planning_worker_.joinable() || planning_active_) return;
    planning_worker_.join();
    if (worker_generation_ != owner_generation_) trajectory_core_->requestCancelCurrent();
    if (generate_header_) {
      Generate::Response response;
      response.failure_code = generated_result_.failure_code;
      response.success = generated_result_.success && worker_generation_ == owner_generation_ &&
        trajectory_core_->follower()->hasFreshJoints(seconds());
      response.message = response.success ? generated_result_.message : "Planning canceled, invalid, or feedback expired: " + generated_result_.message;
      response.request_id = generated_result_.request_id;
      response.theta1_peak_torque = generated_result_.theta1_peak_torque;
      response.theta1_rms_torque = generated_result_.theta1_rms_torque;
      response.minimum_torque_margin = generated_result_.minimum_torque_margin;
      response.trajectory.joint_names = {"theta1", "theta2", "theta3"};
      if (response.success) for (const auto &point : generated_result_.trajectory.points) {
        trajectory_msgs::msg::JointTrajectoryPoint out;
        out.positions.assign(point.positions.begin(), point.positions.begin()+3);
        if (point.velocities.size() == 4) out.velocities.assign(point.velocities.begin(), point.velocities.begin()+3);
        if (point.accelerations.size() == 4) out.accelerations.assign(point.accelerations.begin(), point.accelerations.begin()+3);
        out.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start_sec);
        response.trajectory.points.push_back(out);
      }
      generate_->send_response(*generate_header_, response); generate_header_.reset();
    }
    if (validation_pending_) {
      validation_pending_ = false;
      if (worker_generation_ != owner_generation_ || !trajectory_goal_ || paused_ || ending_) {
        trajectory_core_->requestCancelCurrent();
      } else if (!validation_success_ || !trajectory_core_->follower()->hasFreshJoints(seconds())) {
        begin_ending(false, "Trajectory validation failed: " + validation_failure_.message, true);
      }
    }
  }
  std::thread planning_worker_;
  std::atomic<bool> planning_active_{false};
  bool validation_waiting_ = false, validation_pending_ = false, validation_success_ = false;
  uint64_t owner_generation_ = 0, worker_generation_ = 0;
  path_planner::FollowTrajectoryGoalRequest planned_request_;
  path_planner::FollowTrajectoryActionResult validation_failure_;
  path_planner::GenerateTrajectoryResponse generated_result_;
  std::shared_ptr<rmw_request_id_t> generate_header_;
  std::shared_ptr<path_planner::TrajectoryNode> trajectory_core_;
  rclcpp_action::Server<Trajectory>::SharedPtr trajectory_server_;
  std::shared_ptr<TrajectoryHandle> trajectory_goal_;
  rclcpp::Subscription<Targets>::SharedPtr current_;
  rclcpp::Service<Lease>::SharedPtr lease_;
  rclcpp::Service<catchrobo2026_msgs::srv::GenerateTrajectory>::SharedPtr generate_;
  rclcpp::Service<Trigger>::SharedPtr cancel_trajectory_;
  rclcpp::Client<Trigger>::SharedPtr rebase_;
  std::shared_ptr<rmw_request_id_t> lease_release_header_, cancel_header_, resume_header_;
  Clock::time_point trajectory_started_, ending_deadline_, route_expires_, generate_deadline_, revoked_route_until_;
  std::array<float, 3> final_target_{};
  bool final_ack_ = false, core_finished_ = false;
  Clock::time_point final_ack_at_, feedback_at_owner_;
  bool trajectory_reserved_ = false, ending_ = false, ending_success_ = false, rebase_pending_ = false;
  uint64_t route_token_ = 0, rebase_generation_ = 0, resume_deadline_ns_ = 0, resume_generation_ = 0, trajectory_epoch_ = 0;
  int lease_ms_ = 500;
  double trajectory_timeout_ = 30.0;
  double maximum_tracking_error_ = std::numeric_limits<double>::quiet_NaN();
  std::string ending_message_;

  rclcpp::Subscription<canrpc_bridge::msg::BridgeStatus>::SharedPtr bridge_status_;
  Clock::time_point bridge_status_seen_;
  bool bridge_status_ready_ = false, bridge_status_inhibited_ = false, bridge_status_received_ = false;
  double bridge_status_timeout_sec_ = 2.5;
  std::string bridge_session_, bridge_status_reason_;
  rclcpp::Client<Call>::SharedPtr rpc_;
  rclcpp::Subscription<Targets>::SharedPtr target_;
  rclcpp::Service<Gate>::SharedPtr gate_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<std::array<float, 3>> latest_;
  std::shared_ptr<rmw_request_id_t> pause_header_;
  bool paused_ = true, pending_ = false, faulted_ = false;
  uint64_t token_ = 0, gate_token_ = 0, pause_deadline_ns_ = 0;
  int64_t request_id_ = 0;
  int period_ms_ = 10;
  double timeout_sec_ = 0.15;
  std::string endpoint_;
  Clock::time_point deadline_, last_sent_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTargetClient>());
  rclcpp::shutdown();
  return 0;
}
