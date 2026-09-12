#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <functional>
#include <memory>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "catchrobo2026_msgs/action/execute_sequence.hpp"
#include "catchrobo2026_msgs/action/follow_route.hpp"
#include "catchrobo2026_msgs/srv/check_suction.hpp"
#include "catchrobo2026_msgs/srv/endeffector_control.hpp"
#include "catchrobo2026_msgs/srv/generate_route.hpp"
#include "catchrobo2026_msgs/srv/plan_rotation_group.hpp"
#include "catchrobo2026_msgs/srv/set_sequence_manual.hpp"
#include "catchrobo2026_msgs/srv/wrist_control.hpp"
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_sequence/sequence_config.hpp"

using Trigger = std_srvs::srv::Trigger;
using ExecuteSequence = catchrobo2026_msgs::action::ExecuteSequence;
using FollowRoute = catchrobo2026_msgs::action::FollowRoute;
using GenerateRoute = catchrobo2026_msgs::srv::GenerateRoute;
using PlanRotationGroup = catchrobo2026_msgs::srv::PlanRotationGroup;
using SetSequenceManual = catchrobo2026_msgs::srv::SetSequenceManual;
using WristControl = catchrobo2026_msgs::srv::WristControl;
using PumpControl = catchrobo2026_msgs::srv::PumpControl;
using CheckSuction = catchrobo2026_msgs::srv::CheckSuction;
using EndeffectorControl = catchrobo2026_msgs::srv::EndeffectorControl;
using SequenceGoal = rclcpp_action::ServerGoalHandle<ExecuteSequence>;
using RouteGoal = rclcpp_action::ClientGoalHandle<FollowRoute>;
using Clock = std::chrono::steady_clock;
using catchrobo2026_sequence::SequenceConfig;
using catchrobo2026_sequence::Step;
using catchrobo2026_sequence::StepType;
using catchrobo2026_sequence::MAX_DURATION_SEC;
using catchrobo2026_sequence::MANUAL_CONTROL_ALL;

static_assert(catchrobo2026_sequence::MANUAL_CONTROL_X ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_X);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_Y ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_Y);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_Z ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_Z);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_PHI ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_PHI);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_INITIALIZE ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_INITIALIZE);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_PUMP ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_PUMP);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_ENDEFFECTOR ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_ENDEFFECTOR);
static_assert(catchrobo2026_sequence::MANUAL_CONTROL_ALL ==
  ExecuteSequence::Feedback::MANUAL_CONTROL_ALL);

namespace {
volatile sig_atomic_t stop_requested = 0;
void request_stop(int)
{
    stop_requested = 1;
}
}  // namespace

class SequenceNode : public rclcpp::Node
{
public:
    SequenceNode() : Node("sequence_node")
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        const auto default_file = ament_index_cpp::get_package_share_directory(
            "catchrobo2026_sequence") + "/config/sequences.yaml";
        config_file_ = declare_parameter<std::string>("sequence_file", default_file, descriptor);
        team_ = declare_parameter<std::string>("team", "", descriptor);
        debug_ = declare_parameter<bool>("debug", false, descriptor);
        service_timeout_ = declare_parameter<double>("service_timeout_sec", 3.0, descriptor);
        route_timeout_ = declare_parameter<double>("route_timeout_sec", 30.0, descriptor);
        sequence_timeout_ = declare_parameter<double>("sequence_timeout_sec", 120.0, descriptor);
        stop_timeout_ = declare_parameter<double>("stop_timeout_sec", 3.0, descriptor);
        if (team_ != "red" && team_ != "blue") {
            throw std::invalid_argument("team must be explicitly set to red or blue");
        }
        for (const auto value : {service_timeout_, route_timeout_, sequence_timeout_, stop_timeout_}) {
            if (!std::isfinite(value) || value <= 0.0 || value > MAX_DURATION_SEC) {
                throw std::invalid_argument("timeouts must be finite and in (0, 86400] seconds");
            }
        }
        config_ = std::make_unique<SequenceConfig>(SequenceConfig::load(config_file_));
        initialization_ = create_client<Trigger>("request_initialization");
        planner_ = create_client<GenerateRoute>("generate_route");
        group_planner_ = create_client<PlanRotationGroup>("plan_rotation_group");
        wrist_controller_ = create_client<WristControl>("wrist_control");
        pump_writer_ = create_client<PumpControl>("set_pump_state");
        suction_checker_ = create_client<CheckSuction>("check_suction");
        endeffector_ = create_client<EndeffectorControl>("set_endeffector_state");
        follower_ = rclcpp_action::create_client<FollowRoute>(this, "follow_route");
        manual_control_ = create_service<SetSequenceManual>("set_sequence_manual",
            [this](SetSequenceManual::Request::SharedPtr request,
                   SetSequenceManual::Response::SharedPtr response) {
                set_manual(*request, *response);
            });
        server_ = rclcpp_action::create_server<ExecuteSequence>(
            this, "execute_sequence",
            [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteSequence::Goal> goal) {
                if (goal_ || faulted_ || stop_requested || goal->control_epoch == 0 || goal->step_id == 0 ||
                    goal->collector_mask == 0 || goal->collector_mask > 7 ||
                    (goal->kind != ExecuteSequence::Goal::PICK &&
                     goal->kind != ExecuteSequence::Goal::PLACE &&
                     goal->kind != ExecuteSequence::Goal::START &&
                     goal->kind != ExecuteSequence::Goal::INITIALIZE &&
                     goal->kind != ExecuteSequence::Goal::END) ||
                    ((goal->kind == ExecuteSequence::Goal::START ||
                      goal->kind == ExecuteSequence::Goal::INITIALIZE ||
                      goal->kind == ExecuteSequence::Goal::END) && goal->collector_mask != 7) ||
                    (goal->kind == ExecuteSequence::Goal::PICK &&
                     (goal->row > 3 || goal->column < 1 || goal->column > 4)) ||
                    (goal->kind == ExecuteSequence::Goal::PLACE &&
                     (goal->box > 3 || goal->box_column > 1))) {
                    RCLCPP_WARN(get_logger(), "Sequence rejected: busy, faulted or invalid UI request");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<SequenceGoal>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<SequenceGoal> goal) {start(goal);});
        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
        RCLCPP_INFO(get_logger(), "Sequences: %s, team=%s, debug=%s", config_file_.c_str(),
                    team_.c_str(), debug_ ? "true" : "false");
    }

    void run()
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(get_node_base_interface());
        while (rclcpp::ok() && !stop_requested) {
            executor.spin_once(std::chrono::milliseconds(20));
        }
        if (goal_ && rclcpp::ok()) {
            begin_stop("sequence node shutting down");
            while (goal_ && rclcpp::ok()) {
                executor.spin_once(std::chrono::milliseconds(20));
            }
        }
    }

private:
    enum class Phase {
        READY, WAIT_PLAN, PLAN, WAIT_FOLLOW, FOLLOW_GOAL, FOLLOWING,
        WAIT_GROUP_PLAN, GROUP_PLAN, WAIT_WRIST_BEGIN, WRIST_BEGIN,
        WAIT_WRIST_END, WRIST_END,
        WAIT_PHI_TRAVEL, PHI_TRAVEL,
        WAIT_PUMP_SET, PUMP_SET, WAIT_END, END, WAIT_INITIALIZE, INITIALIZE, DELAY,
        WAIT_SUCTION_SERVICE, SUCTION_RESULT, MANUAL_WAIT
    };

    static Clock::time_point after(double seconds)
    {
        if (!std::isfinite(seconds) || seconds < 0.0 || seconds > MAX_DURATION_SEC) {
            throw std::invalid_argument("duration must be in [0, 86400] seconds");
        }
        return Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(seconds));
    }

    void publish_feedback()
    {
        auto feedback = std::make_shared<ExecuteSequence::Feedback>();
        feedback->step_index = static_cast<uint32_t>(index_);
        feedback->manual_token = manual_token_;
        feedback->manual_allowed_controls = phase_ == Phase::MANUAL_WAIT ?
            manual_allowed_controls_ : 0u;
        feedback->phase = manual_requested_ && phase_ != Phase::MANUAL_WAIT ?
            "manual requested: " + phase_name_ : phase_name_;
        goal_->publish_feedback(feedback);
        feedback_at_ = Clock::now();
    }

    void transition(Phase phase, const std::string &name, double timeout)
    {
        phase_ = phase;
        phase_name_ = name;
        deadline_ = after(timeout);
        publish_feedback();
    }

    void set_manual(const SetSequenceManual::Request &request,
                    SetSequenceManual::Response &response)
    {
        if (!goal_ || stopping_ || faulted_ || goal_->is_canceling() ||
            request.control_epoch != goal_->get_goal()->control_epoch ||
            request.step_id != goal_->get_goal()->step_id) {
            response.message = "no matching active sequence available for manual control";
            return;
        }
        if (request.manual) {
            manual_requested_ = true;
            if (!try_enter_manual()) {
                publish_feedback();
            }
            response.message = phase_ == Phase::MANUAL_WAIT ?
                "manual waiting" : "manual requested; waiting for the active operation";
        } else {
            if (request.manual_token != manual_token_) {
                response.message = "manual token does not match the current wait";
                return;
            }
            manual_requested_ = false;
            if (phase_ == Phase::MANUAL_WAIT) {
                sequence_deadline_ += Clock::now() - manual_started_;
                if (manual_consumes_step_) {
                    ++index_;
                }
                manual_allowed_controls_ = 0u;
                transition(manual_resume_phase_,
                    manual_resume_phase_ == Phase::DELAY ? "waiting" : "ready",
                    manual_resume_phase_ == Phase::DELAY ? manual_remaining_sec_ : service_timeout_);
                response.message = "automatic sequence resumed";
            } else {
                publish_feedback();
                response.message = "manual request cleared";
            }
        }
        response.success = true;
    }

    bool try_enter_manual()
    {
        if (!manual_requested_ || group_needs_release_ || suction_active_ ||
            service_pending_ || route_pending_ || route_goal_ || wrist_end_pending_ ||
            (phase_ != Phase::READY && phase_ != Phase::DELAY)) {
            return false;
        }
        manual_resume_phase_ = phase_;
        manual_consumes_step_ = phase_ == Phase::READY && index_ < steps_.size() &&
            steps_[index_].type == StepType::MANUAL;
        manual_remaining_sec_ = phase_ == Phase::DELAY ?
            std::max(0.0, std::chrono::duration<double>(deadline_ - Clock::now()).count()) : 0.0;
        manual_started_ = Clock::now();
        ++manual_token_;
        manual_allowed_controls_ = manual_consumes_step_ ?
            steps_[index_].manual_allowed_controls : MANUAL_CONTROL_ALL;
        std::string label = "manual waiting";
        if (manual_consumes_step_ && !steps_[index_].message.empty()) {
            label += ": " + steps_[index_].message;
        }
        transition(Phase::MANUAL_WAIT, label, 0.0);
        RCLCPP_INFO(get_logger(), "Manual control at step %zu: %s", index_, label.c_str());
        return true;
    }

    void start(const std::shared_ptr<SequenceGoal> &goal)
    {
        goal_ = goal;
        const auto request = goal_->get_goal();
        const bool lifecycle = request->kind == ExecuteSequence::Goal::START ||
            request->kind == ExecuteSequence::Goal::INITIALIZE ||
            request->kind == ExecuteSequence::Goal::END;
        if (lifecycle || request->control_epoch != pending_epoch_ ||
            request->step_id <= pending_step_id_) {
            clear_pending_waypoints();
        }
        staged_waypoints_.clear();
        index_ = 0;
        stopping_ = false;
        manual_requested_ = false;
        manual_consumes_step_ = false;
        manual_token_ = 0;
        manual_allowed_controls_ = 0u;
        stop_message_.clear();
        cancel_sent_ = false;
        group_routes_.clear();
        planned_wrist_angles_.clear();
        planned_phi_angles_.clear();
        sequence_deadline_ = after(sequence_timeout_);
        try {
            // Compile the entire action before sending any hardware command.
            auto candidate = debug_ ? SequenceConfig::load(config_file_) : *config_;
            const bool pick = request->kind == ExecuteSequence::Goal::PICK;
            switch (request->kind) {
            case ExecuteSequence::Goal::INITIALIZE:
                steps_ = candidate.compile_initialization();
                break;
            case ExecuteSequence::Goal::END:
                steps_ = candidate.compile_end();
                break;
            case ExecuteSequence::Goal::START:
                steps_ = candidate.compile_start();
                break;
            default:
                steps_ = candidate.compile(team_, pick ? "pick" : "place",
                    pick ? request->row : request->box,
                    pick ? request->column : request->box_column);
                break;
            }
            if (steps_.empty() && !lifecycle) {
                throw std::runtime_error("the selected sequence has no steps");
            }
            auto prepared = catchrobo2026_sequence::prepare_sequence(
                std::move(steps_), pending_waypoints_);
            steps_ = std::move(prepared.steps);
            staged_waypoints_ = std::move(prepared.deferred_waypoints);
            clear_pending_waypoints();
            if (prepared.consumed_pending_waypoints != 0 || !staged_waypoints_.empty()) {
                RCLCPP_INFO(get_logger(),
                    "Deferred waypoints: %zu added to the first MOVE, %zu retained after success",
                    prepared.consumed_pending_waypoints, staged_waypoints_.size());
            }
            active_route_timeout_ = candidate.route_timeout_sec().value_or(route_timeout_);
            if (debug_) {
                *config_ = std::move(candidate);
            }
            transition(Phase::READY, "ready", service_timeout_);
        } catch (const std::exception &error) {
            finish(false, std::string("configuration: ") + error.what());
        }
    }

    void begin_stop(const std::string &message)
    {
        if (stopping_) {
            return;
        }
        clear_pending_waypoints();
        staged_waypoints_.clear();
        stopping_ = true;
        manual_requested_ = false;
        manual_allowed_controls_ = 0u;
        abandon_suction_check();
        stop_message_ = message;
        stop_deadline_ = after(stop_timeout_);
        RCLCPP_WARN(get_logger(), "Stopping sequence: %s", message.c_str());
    }

    void finish(bool success, const std::string &message)
    {
        abandon_suction_check();
        auto result = std::make_shared<ExecuteSequence::Result>();
        result->success = success && !goal_->is_canceling();
        result->message = message;
        if (result->success &&
            (goal_->get_goal()->kind == ExecuteSequence::Goal::PICK ||
             goal_->get_goal()->kind == ExecuteSequence::Goal::PLACE)) {
            pending_waypoints_ = std::move(staged_waypoints_);
            pending_epoch_ = goal_->get_goal()->control_epoch;
            pending_step_id_ = goal_->get_goal()->step_id;
        } else {
            clear_pending_waypoints();
        }
        staged_waypoints_.clear();
        if (goal_->is_canceling()) {
            goal_->canceled(result);
        } else if (success) {
            goal_->succeed(result);
        } else {
            goal_->abort(result);
        }
        RCLCPP_INFO(get_logger(), "Sequence %s: %s", result->success ? "completed" : "stopped",
                    message.c_str());
        goal_.reset();
        steps_.clear();
        manual_allowed_controls_ = 0u;
    }

    void clear_pending_waypoints()
    {
        pending_waypoints_.clear();
        pending_epoch_ = 0;
        pending_step_id_ = 0;
    }

    void abandon_suction_check()
    {
        // Monitoring is read-only and must not delay motion cancellation.
        ++suction_generation_;
        if (abandon_suction_) {
            abandon_suction_();
            abandon_suction_ = {};
        }
        suction_pending_ = false;
        suction_active_ = false;
        suction_reply_.reset();
    }

    void start_suction_check()
    {
        if (suction_active_ || suction_pending_) {
            throw std::logic_error("a suction check is already active");
        }
        auto value = std::make_shared<CheckSuction::Request>();
        value->collector_mask = goal_->get_goal()->collector_mask;
        value->timeout_sec = steps_[index_].seconds;
        // The sensor deadline is independent of ordinary command deadlines.
        suction_deadline_ = after(value->timeout_sec) +
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(service_timeout_));
        suction_active_ = true;
        suction_pending_ = true;
        suction_reply_.reset();
        const auto generation = ++suction_generation_;
        auto pending = suction_checker_->async_send_request(value,
            [this, generation](rclcpp::Client<CheckSuction>::SharedFuture response) {
                if (generation != suction_generation_) {
                    return;
                }
                suction_pending_ = false;
                abandon_suction_ = {};
                if (interrupted()) {
                    return;
                }
                try {
                    const auto reply = response.get();
                    const auto mask = goal_->get_goal()->collector_mask;
                    if (reply->suction_mask > 7 || (reply->success &&
                        (reply->timed_out || (reply->suction_mask & mask) != mask))) {
                        begin_stop("invalid suction check response");
                        return;
                    }
                    if (!reply->success && !reply->timed_out) {
                        begin_stop("suction check rejected: " + reply->message);
                        return;
                    }
                    suction_reply_ = reply;
                    RCLCPP_INFO(get_logger(),
                        "Suction check: success=%s, mask=%u, pressure=[%d,%d,%d]: %s",
                        reply->success ? "true" : "false", reply->suction_mask,
                        reply->pressure[0], reply->pressure[1], reply->pressure[2],
                        reply->message.c_str());
                } catch (const std::exception &error) {
                    begin_stop(std::string("suction check response: ") + error.what());
                }
            });
        const auto id = pending.request_id;
        abandon_suction_ = [client = suction_checker_, id]() {client->remove_pending_request(id);};
        command_done(true, "");
    }

    bool interrupted() const
    {
        return !goal_ || stopping_ || goal_->is_canceling();
    }

    template<typename Service, typename Callback>
    void request(const typename rclcpp::Client<Service>::SharedPtr &client,
                 const std::shared_ptr<typename Service::Request> &value, Callback callback,
                 bool handle_interruption = false)
    {
        service_pending_ = true;
        auto pending = client->async_send_request(value,
            [this, callback, handle_interruption](typename rclcpp::Client<Service>::SharedFuture response) {
                service_pending_ = false;
                abandon_request_ = {};
                if (!goal_ || (interrupted() && !handle_interruption)) {
                    return;
                }
                try {
                    callback(response.get());
                } catch (const std::exception &error) {
                    begin_stop(std::string("service response: ") + error.what());
                }
            });
        const auto id = pending.request_id;
        abandon_request_ = [client, id]() {client->remove_pending_request(id);};
    }

    void tick()
    {
        try {
            tick_impl();
        } catch (const std::exception &error) {
            if (goal_) {
                begin_stop(std::string("ROS operation: ") + error.what());
            }
        }
    }

    void tick_impl()
    {
        if (!goal_) {
            return;
        }
        if (goal_->is_canceling()) {
            begin_stop("UI canceled the sequence");
        } else if (phase_ != Phase::MANUAL_WAIT && Clock::now() >= sequence_deadline_) {
            begin_stop("sequence timeout");
        }
        if (stopping_) {
            if (route_goal_ && !cancel_sent_) {
                cancel_sent_ = true;
                follower_->async_cancel_goal(route_goal_);
            }
            // Release the wrist even if a follower cancellation reply is delayed.
            if (group_needs_release_ && !group_begin_pending_ && !wrist_end_sent_ &&
                wrist_controller_->service_is_ready()) {
                end_sequence_group();
            }
            if (!service_pending_ && !wrist_end_pending_ && !route_pending_ &&
                !route_goal_ && !group_needs_release_) {
                finish(false, stop_message_);
            } else if (Clock::now() >= stop_deadline_) {
                // A lost reply can conceal an active command; require a restart.
                faulted_ = true;
                if (abandon_request_) {
                    abandon_request_();
                    abandon_request_ = {};
                }
                if (abandon_wrist_end_) {
                    abandon_wrist_end_();
                    abandon_wrist_end_ = {};
                }
                finish(false, stop_message_ + "; stop unconfirmed, sequencer restart required");
            }
            return;
        }
        if (manual_requested_ && Clock::now() - feedback_at_ >= std::chrono::milliseconds(200)) {
            // Repeat manual status in case the initial feedback preceded goal acceptance.
            publish_feedback();
        }
        if (phase_ == Phase::MANUAL_WAIT || try_enter_manual()) {
            return;
        }
        if (suction_pending_ && Clock::now() >= suction_deadline_) {
            begin_stop("suction check response timeout");
            return;
        }
        if (phase_ != Phase::DELAY && phase_ != Phase::SUCTION_RESULT &&
            Clock::now() >= deadline_) {
            begin_stop(phase_name_ + " timeout");
            return;
        }
        switch (phase_) {
        case Phase::READY:
            next_step();
            break;
        case Phase::WAIT_PLAN:
            if (planner_->service_is_ready()) {
                transition(Phase::PLAN, "planning", service_timeout_);
                request<GenerateRoute>(planner_, generate_request_,
                    [this](GenerateRoute::Response::SharedPtr reply) {
                        if (!reply->success || reply->path.poses.empty()) {
                            begin_stop("route generation failed or returned an empty path");
                            return;
                        }
                        planned_path_ = reply->path;
                        transition(Phase::WAIT_FOLLOW, "waiting for follower", service_timeout_);
                    });
            }
            break;
        case Phase::WAIT_GROUP_PLAN:
            if (group_planner_->service_is_ready()) {
                transition(Phase::GROUP_PLAN, "planning sequence group", service_timeout_);
                request<PlanRotationGroup>(group_planner_, group_request_,
                    [this](PlanRotationGroup::Response::SharedPtr reply) {
                        accept_rotation_plan(*reply);
                    });
            }
            break;
        case Phase::WAIT_WRIST_BEGIN:
            if (wrist_controller_->service_is_ready()) {
                begin_sequence_group();
            }
            break;
        case Phase::WAIT_WRIST_END:
            if (wrist_controller_->service_is_ready()) {
                end_sequence_group();
            }
            break;
        case Phase::WAIT_PHI_TRAVEL:
            if (wrist_controller_->service_is_ready()) {
                auto value = std::make_shared<WristControl::Request>();
                const auto &step = steps_[index_];
                value->operation = step.type == StepType::PHI_TRAVEL_START ?
                    WristControl::Request::PHI_BEGIN : WristControl::Request::PHI_END;
                value->group_id = group_id_;
                value->limit_phi_travel = step.max_phi_travel.has_value();
                value->max_phi_travel = step.max_phi_travel.value_or(0.0);
                transition(Phase::PHI_TRAVEL, "setting phi travel interval", service_timeout_);
                request<WristControl>(wrist_controller_, value,
                    [this](WristControl::Response::SharedPtr reply) {
                        command_done(reply->success, "phi travel interval rejected: " + reply->message);
                    });
            }
            break;
        case Phase::WAIT_FOLLOW:
            if (follower_->action_server_is_ready()) {
                follow();
            }
            break;
        case Phase::WAIT_PUMP_SET:
            if (pump_writer_->service_is_ready()) {
                transition(Phase::PUMP_SET, "setting pump", service_timeout_);
                request<PumpControl>(pump_writer_, pump_request_,
                    [this](PumpControl::Response::SharedPtr reply) {
                        command_done(reply->success, "pump command rejected");
                    });
            }
            break;
        case Phase::WAIT_SUCTION_SERVICE:
            if (suction_checker_->service_is_ready()) {
                start_suction_check();
            }
            break;
        case Phase::SUCTION_RESULT:
            if (suction_reply_) {
                suction_active_ = false;
                command_done(true, "");
            }
            break;
        case Phase::WAIT_END:
            if (endeffector_->service_is_ready()) {
                auto value = std::make_shared<EndeffectorControl::Request>();
                value->command = steps_[index_].command;
                transition(Phase::END, "setting endeffector", service_timeout_);
                request<EndeffectorControl>(endeffector_, value,
                    [this](EndeffectorControl::Response::SharedPtr reply) {
                        command_done(reply->success, "endeffector command rejected");
                    });
            }
            break;
        case Phase::WAIT_INITIALIZE:
            if (initialization_->service_is_ready()) {
                transition(Phase::INITIALIZE, "requesting initialization", service_timeout_);
                request<Trigger>(initialization_, std::make_shared<Trigger::Request>(),
                    [this](Trigger::Response::SharedPtr reply) {
                        command_done(reply->success, "initialization rejected: " + reply->message);
                    });
            }
            break;
        case Phase::DELAY:
            if (Clock::now() >= deadline_) {
                command_done(true, "");
            }
            break;
        default:
            break;
        }
    }

    void command_done(bool success, const std::string &message)
    {
        if (!success) {
            begin_stop(message);
            return;
        }
        ++index_;
        transition(Phase::READY, "ready", service_timeout_);
    }

    void next_step()
    {
        if (index_ > steps_.size()) {
            throw std::logic_error("sequence jump is out of range");
        }
        if (index_ == steps_.size()) {
            if (group_needs_release_) {
                begin_stop("sequence group was not released");
                return;
            }
            if (suction_active_) {
                throw std::logic_error("sequence ended before waiting for its suction check");
            }
            finish(true, staged_waypoints_.empty() ? "sequence completed" :
                "sequence completed; " + std::to_string(staged_waypoints_.size()) +
                " waypoints deferred to the next MOVE");
            return;
        }
        const auto &step = steps_[index_];
        switch (step.type) {
        case StepType::MOVE:
            if (group_needs_release_) {
                const auto &route = group_routes_.at(index_);
                route_end_index_ = route.end_index;
                planned_path_ = route.route.path;
                planned_wrist_angles_ = route.route.wrist_angles;
                planned_phi_angles_ = route.route.phi_angles;
                transition(Phase::WAIT_FOLLOW, "waiting for follower", service_timeout_);
                break;
            }
            planned_wrist_angles_.clear();
            planned_phi_angles_.clear();
            generate_request_ = std::make_shared<GenerateRoute::Request>();
            generate_request_->use_explicit_waypoints = true;
            route_end_index_ = index_;
            {
                auto append_waypoint = [this](const catchrobo2026_sequence::Pose &point) {
                    geometry_msgs::msg::Pose waypoint;
                    waypoint.position.x = point[0] / 1000.0;
                    waypoint.position.y = point[1] / 1000.0;
                    waypoint.position.z = point[2] / 1000.0;
                    waypoint.orientation.z = std::sin(point[3] / 2.0);
                    waypoint.orientation.w = std::cos(point[3] / 2.0);
                    generate_request_->waypoints.push_back(waypoint);
                };
                // Configuration validation guarantees a final, non-waypoint MOVE.
                while (true) {
                    const auto &move = steps_[route_end_index_];
                    for (const auto &point : move.waypoints) {
                        append_waypoint(point);
                    }
                    if (!move.waypoint) {
                        break;
                    }
                    append_waypoint(move.pose);
                    ++route_end_index_;
                }
            }
            {
                const auto &target = steps_[route_end_index_].pose;
                generate_request_->x = target[0];
                generate_request_->y = target[1];
                generate_request_->z = target[2];
                generate_request_->phi = target[3];
                RCLCPP_INFO(get_logger(),
                    "MOVE steps %zu..%zu: %zu waypoints -> [%.2f, %.2f, %.2f, %.3f], timeout=%.2fs",
                    index_, route_end_index_, generate_request_->waypoints.size(),
                    target[0], target[1], target[2], target[3], active_route_timeout_);
            }
            transition(Phase::WAIT_PLAN, "waiting for planner", service_timeout_);
            break;
        case StepType::PUMP:
            {
                const auto mask = goal_->get_goal()->collector_mask;
                const auto command = static_cast<int8_t>(step.command);
                pump_request_ = std::make_shared<PumpControl::Request>();
                pump_request_->left = (mask & 1u) ? command : PumpControl::Request::OFF;
                pump_request_->center = (mask & 2u) ? command : PumpControl::Request::OFF;
                pump_request_->right = (mask & 4u) ? command : PumpControl::Request::OFF;
            }
            transition(Phase::WAIT_PUMP_SET, "waiting for pump", service_timeout_);
            break;
        case StepType::ENDEFFECTOR:
            transition(Phase::WAIT_END, "waiting for endeffector", service_timeout_);
            break;
        case StepType::INITIALIZE:
            transition(Phase::WAIT_INITIALIZE, "waiting for initialization", service_timeout_);
            break;
        case StepType::WAIT:
            transition(Phase::DELAY, "waiting", step.seconds);
            break;
        case StepType::MANUAL:
            manual_requested_ = true;
            if (!try_enter_manual()) {
                throw std::logic_error("manual step requires an inactive sequence group and suction check");
            }
            break;
        case StepType::SEQUENCE_START:
            prepare_sequence_group();
            break;
        case StepType::SEQUENCE_END:
            transition(Phase::WAIT_WRIST_END, "waiting to release sequence group", service_timeout_);
            break;
        case StepType::PHI_TRAVEL_START:
        case StepType::PHI_TRAVEL_END:
            transition(Phase::WAIT_PHI_TRAVEL, "waiting to set phi travel interval", service_timeout_);
            break;
        case StepType::SUCTION_CHECK_START:
            transition(Phase::WAIT_SUCTION_SERVICE, "waiting for suction checker", service_timeout_);
            break;
        case StepType::SUCTION_CHECK_WAIT:
            if (!suction_active_) {
                throw std::logic_error("no suction check to wait for");
            }
            transition(Phase::SUCTION_RESULT, "waiting for suction result", service_timeout_);
            break;
        case StepType::IF_SUCTION:
            if (suction_active_ || !suction_reply_) {
                throw std::logic_error("suction condition requires a completed wait");
            }
            index_ = suction_reply_->success == step.condition_success ? index_ + 1 : step.jump_index;
            transition(Phase::READY, "branching on suction result", service_timeout_);
            break;
        case StepType::JUMP:
            index_ = step.jump_index;
            transition(Phase::READY, "jumping", service_timeout_);
            break;
        case StepType::FAIL:
            begin_stop(step.message);
            break;
        }
    }

    static geometry_msgs::msg::Pose route_pose(const catchrobo2026_sequence::Pose &point)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = point[0] / 1000.0;
        pose.position.y = point[1] / 1000.0;
        pose.position.z = point[2] / 1000.0;
        pose.orientation.z = std::sin(point[3] / 2.0);
        pose.orientation.w = std::cos(point[3] / 2.0);
        return pose;
    }

    void prepare_sequence_group()
    {
        group_routes_.clear();
        group_request_ = std::make_shared<PlanRotationGroup::Request>();
        const auto &group = steps_[index_];
        group_request_->allow_wrist_reversal = !group.rotation_group;
        group_request_->limit_phi_travel = group.max_phi_travel.has_value();
        group_request_->max_phi_travel = group.max_phi_travel.value_or(0.0);
        // Resolve every route before executing any command inside the group.
        size_t cursor = index_ + 1;
        for (; cursor < steps_.size() && steps_[cursor].type != StepType::SEQUENCE_END; ++cursor) {
            if (steps_[cursor].type != StepType::MOVE) {
                continue;
            }
            const size_t first = cursor;
            std::vector<std::pair<size_t, Step>> internal_phi_events;
            size_t route_target = 0;
            while (true) {
                const auto &move = steps_.at(cursor);
                for (const auto &point : move.waypoints) {
                    group_request_->targets.push_back(route_pose(point));
                    ++route_target;
                }
                group_request_->targets.push_back(route_pose(move.pose));
                ++route_target;
                if (!move.waypoint) {
                    break;
                }
                ++cursor;
                while (cursor < steps_.size() &&
                       (steps_[cursor].type == StepType::PHI_TRAVEL_START ||
                        steps_[cursor].type == StepType::PHI_TRAVEL_END)) {
                    internal_phi_events.emplace_back(route_target, steps_[cursor]);
                    ++cursor;
                }
            }
            group_request_->route_ends.push_back(
                static_cast<uint32_t>(group_request_->targets.size()));
            group_routes_[first].end_index = cursor;
            group_routes_[first].target_count = route_target;
            group_routes_[first].phi_events = std::move(internal_phi_events);
        }
        if (cursor == steps_.size() || group_routes_.empty()) {
            throw std::runtime_error("invalid sequence group boundaries");
        }
        for (const auto &interval : catchrobo2026_sequence::collect_phi_travel_intervals(
                steps_, index_, cursor)) {
            catchrobo2026_msgs::msg::PhiTravelInterval value;
            value.start_target = static_cast<uint32_t>(interval.start_target);
            value.end_target = static_cast<uint32_t>(interval.end_target);
            value.max_phi_travel = interval.max_phi_travel;
            group_request_->phi_travel_intervals.push_back(value);
        }
        transition(Phase::WAIT_GROUP_PLAN, "waiting for sequence group planner", service_timeout_);
    }

    void accept_rotation_plan(const PlanRotationGroup::Response &reply)
    {
        if (!reply.success || reply.routes.size() != group_routes_.size() ||
            (reply.direction != 1 && reply.direction != -1 &&
                !(reply.direction == 0 && group_request_->allow_wrist_reversal))) {
            begin_stop("sequence group planning failed: " + reply.message);
            return;
        }
        if (!std::isfinite(reply.phi_travel) || reply.phi_travel < 0.0 ||
            (group_request_->limit_phi_travel &&
                reply.phi_travel > group_request_->max_phi_travel + 1e-5)) {
            begin_stop("sequence group planner exceeded the total phi travel limit");
            return;
        }
        size_t offset = 0;
        if (reply.interval_phi_travel.size() != group_request_->phi_travel_intervals.size()) {
            begin_stop("sequence group planner omitted phi interval results");
            return;
        }
        for (size_t i = 0; i < reply.interval_phi_travel.size(); ++i) {
            const double amount = reply.interval_phi_travel[i];
            if (!std::isfinite(amount) || amount < 0.0 ||
                amount > group_request_->phi_travel_intervals[i].max_phi_travel + 1e-5) {
                begin_stop("sequence group planner exceeded a phi interval limit");
                return;
            }
        }
        for (auto &[first, cached] : group_routes_) {
            (void)first;
            const auto &route = reply.routes[offset++];
            if (route.path.poses.empty() || route.wrist_angles.size() != route.path.poses.size() ||
                (!route.phi_angles.empty() && route.phi_angles.size() != route.path.poses.size()) ||
                route.target_sample_indices.size() != cached.target_count ||
                route.target_sample_indices.empty() ||
                route.target_sample_indices.front() == 0 ||
                route.target_sample_indices.back() != route.path.poses.size() - 1 ||
                std::adjacent_find(route.target_sample_indices.begin(),
                    route.target_sample_indices.end(), std::greater_equal<uint32_t>()) !=
                    route.target_sample_indices.end()) {
                begin_stop("sequence group planner returned an invalid route");
                return;
            }
            cached.route = route;
        }
        group_direction_ = reply.direction;
        const auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        group_id_ = std::max(group_id_ + 1, static_cast<uint64_t>(timestamp));
        wrist_end_sent_ = false;
        transition(Phase::WAIT_WRIST_BEGIN, "waiting to start sequence group", service_timeout_);
    }

    void begin_sequence_group()
    {
        const auto &first = group_routes_.begin()->second.route;
        auto value = std::make_shared<WristControl::Request>();
        value->operation = WristControl::Request::BEGIN;
        value->group_id = group_id_;
        value->direction = group_direction_;
        value->target = first.path.poses.front();
        value->wrist_angle = first.wrist_angles.front();
        value->limit_phi_travel = group_request_->limit_phi_travel;
        value->max_phi_travel = group_request_->max_phi_travel;
        transition(Phase::WRIST_BEGIN, "starting sequence group", service_timeout_);
        // Cancellation can arrive before the BEGIN reply, so retain its lease.
        group_needs_release_ = true;
        group_begin_pending_ = true;
        request<WristControl>(wrist_controller_, value,
            [this](WristControl::Response::SharedPtr reply) {
                group_begin_pending_ = false;
                if (!reply->success) {
                    group_needs_release_ = false;
                    begin_stop("sequence group start rejected: " + reply->message);
                } else if (!interrupted()) {
                    RCLCPP_INFO(get_logger(),
                        "Sequence group %llu: fourth joint direction=%d, phi limit=%s %.6f rad",
                        static_cast<unsigned long long>(group_id_), group_direction_,
                        group_request_->limit_phi_travel ? "enabled" : "disabled",
                        group_request_->max_phi_travel);
                    command_done(true, "");
                }
            }, true);
    }

    void end_sequence_group()
    {
        auto value = std::make_shared<WristControl::Request>();
        value->operation = WristControl::Request::END;
        value->group_id = group_id_;
        wrist_end_sent_ = true;
        if (!stopping_) {
            transition(Phase::WRIST_END, "releasing sequence group", service_timeout_);
        }
        // Release independently of unrelated service replies during cancellation.
        wrist_end_pending_ = true;
        auto pending = wrist_controller_->async_send_request(value,
            [this](rclcpp::Client<WristControl>::SharedFuture response) {
                wrist_end_pending_ = false;
                abandon_wrist_end_ = {};
                if (!goal_) return;
                try {
                    const auto reply = response.get();
                    if (!reply->success) {
                        faulted_ = true;
                        begin_stop("sequence group release rejected: " + reply->message);
                        return;
                    }
                    group_needs_release_ = false;
                    group_routes_.clear();
                    planned_wrist_angles_.clear();
                    planned_phi_angles_.clear();
                    if (!interrupted()) {
                        command_done(true, "");
                    }
                } catch (const std::exception &error) {
                    begin_stop(std::string("sequence group release: ") + error.what());
                }
            });
        const auto id = pending.request_id;
        abandon_wrist_end_ = [this, id]() {wrist_controller_->remove_pending_request(id);};
    }

    void follow()
    {
        FollowRoute::Goal goal;
        goal.start = true;
        goal.path = planned_path_;
        if (group_needs_release_) {
            goal.rotation_group_id = group_id_;
            goal.wrist_direction = group_direction_;
            goal.wrist_angles = planned_wrist_angles_;
            goal.phi_angles = planned_phi_angles_;
            const auto &cached = group_routes_.at(index_);
            for (const auto &[target, step] : cached.phi_events) {
                catchrobo2026_msgs::msg::PhiTravelEvent event;
                event.sample_index = cached.route.target_sample_indices.at(target - 1);
                event.operation = step.type == StepType::PHI_TRAVEL_START ?
                    catchrobo2026_msgs::msg::PhiTravelEvent::BEGIN :
                    catchrobo2026_msgs::msg::PhiTravelEvent::END;
                event.limit_phi_travel = step.max_phi_travel.has_value();
                event.max_phi_travel = step.max_phi_travel.value_or(0.0);
                goal.phi_travel_events.push_back(event);
            }
        }
        route_pending_ = true;
        cancel_sent_ = false;
        transition(Phase::FOLLOW_GOAL, "sending route", service_timeout_);
        rclcpp_action::Client<FollowRoute>::SendGoalOptions options;
        options.goal_response_callback = [this](RouteGoal::SharedPtr accepted) {
            route_pending_ = false;
            route_goal_ = accepted;
            try {
                if (!goal_) {
                    if (accepted) {
                        follower_->async_cancel_goal(accepted);
                    }
                    return;
                }
                if (interrupted()) {
                    return;
                }
                if (!accepted) {
                    begin_stop("follower rejected the route");
                    return;
                }
                transition(Phase::FOLLOWING, "following route", active_route_timeout_);
            } catch (const std::exception &error) {
                if (goal_) {
                    begin_stop(std::string("route response: ") + error.what());
                } else {
                    faulted_ = true;
                    RCLCPP_ERROR(get_logger(), "Late route cancellation failed: %s", error.what());
                }
            }
        };
        options.result_callback = [this](const RouteGoal::WrappedResult &result) {
            route_goal_.reset();
            if (interrupted()) {
                return;
            }
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
                !result.result || !result.result->success) {
                begin_stop("route following failed");
                return;
            }
            try {
                index_ = route_end_index_ + 1;
                transition(Phase::READY, "ready", service_timeout_);
            } catch (const std::exception &error) {
                begin_stop(std::string("route result: ") + error.what());
            }
        };
        follower_->async_send_goal(goal, options);
    }

    std::string config_file_, team_, phase_name_, stop_message_;
    bool debug_{false}, stopping_{false}, faulted_{false};
    bool manual_requested_{false}, manual_consumes_step_{false};
    uint64_t manual_token_{0};
    uint32_t manual_allowed_controls_{0u};
    bool service_pending_{false}, route_pending_{false}, cancel_sent_{false};
    bool suction_pending_{false}, suction_active_{false};
    uint64_t suction_generation_{0};
    double service_timeout_, route_timeout_, sequence_timeout_, stop_timeout_;
    double active_route_timeout_{0.0};
    Phase phase_{Phase::READY};
    Phase manual_resume_phase_{Phase::READY};
    double manual_remaining_sec_{0.0};
    Clock::time_point manual_started_;
    Clock::time_point feedback_at_;
    Clock::time_point deadline_, sequence_deadline_, stop_deadline_;
    Clock::time_point suction_deadline_;
    std::unique_ptr<SequenceConfig> config_;
    std::vector<Step> steps_;
    std::vector<catchrobo2026_sequence::Pose> pending_waypoints_, staged_waypoints_;
    uint64_t pending_epoch_{0}, pending_step_id_{0};
    struct GroupRoute {
        size_t end_index{0};
        size_t target_count{0};
        std::vector<std::pair<size_t, Step>> phi_events;
        catchrobo2026_msgs::msg::RotationGroupRoute route;
    };
    std::map<size_t, GroupRoute> group_routes_;
    uint64_t group_id_{0};
    int8_t group_direction_{0};
    bool group_needs_release_{false}, wrist_end_sent_{false};
    bool group_begin_pending_{false}, wrist_end_pending_{false};
    std::vector<double> planned_wrist_angles_;
    std::vector<double> planned_phi_angles_;
    size_t index_{0};
    size_t route_end_index_{0};
    nav_msgs::msg::Path planned_path_;
    GenerateRoute::Request::SharedPtr generate_request_;
    PlanRotationGroup::Request::SharedPtr group_request_;
    PumpControl::Request::SharedPtr pump_request_;
    CheckSuction::Response::SharedPtr suction_reply_;
    std::function<void()> abandon_request_;
    std::function<void()> abandon_wrist_end_;
    std::function<void()> abandon_suction_;
    std::shared_ptr<SequenceGoal> goal_;
    RouteGoal::SharedPtr route_goal_;
    rclcpp::Client<Trigger>::SharedPtr initialization_;
    rclcpp::Client<GenerateRoute>::SharedPtr planner_;
    rclcpp::Client<PlanRotationGroup>::SharedPtr group_planner_;
    rclcpp::Client<WristControl>::SharedPtr wrist_controller_;
    rclcpp::Client<PumpControl>::SharedPtr pump_writer_;
    rclcpp::Client<CheckSuction>::SharedPtr suction_checker_;
    rclcpp::Client<EndeffectorControl>::SharedPtr endeffector_;
    rclcpp_action::Client<FollowRoute>::SharedPtr follower_;
    rclcpp_action::Server<ExecuteSequence>::SharedPtr server_;
    rclcpp::Service<SetSequenceManual>::SharedPtr manual_control_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    int result = 0;
    try {
        std::make_shared<SequenceNode>()->run();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("sequence_node"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
