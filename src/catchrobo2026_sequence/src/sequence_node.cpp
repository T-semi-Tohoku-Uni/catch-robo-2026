#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "catchrobo2026_msgs/action/execute_sequence.hpp"
#include "catchrobo2026_msgs/action/follow_route.hpp"
#include "catchrobo2026_msgs/srv/check_suction.hpp"
#include "catchrobo2026_msgs/srv/endeffector_control.hpp"
#include "catchrobo2026_msgs/srv/generate_route.hpp"
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_sequence/sequence_config.hpp"

using Trigger = std_srvs::srv::Trigger;
using ExecuteSequence = catchrobo2026_msgs::action::ExecuteSequence;
using FollowRoute = catchrobo2026_msgs::action::FollowRoute;
using GenerateRoute = catchrobo2026_msgs::srv::GenerateRoute;
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
        pump_writer_ = create_client<PumpControl>("set_pump_state");
        suction_checker_ = create_client<CheckSuction>("check_suction");
        endeffector_ = create_client<EndeffectorControl>("set_endeffector_state");
        follower_ = rclcpp_action::create_client<FollowRoute>(this, "follow_route");
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
        WAIT_PUMP_SET, PUMP_SET, WAIT_END, END, WAIT_INITIALIZE, INITIALIZE, DELAY,
        WAIT_SUCTION_SERVICE, SUCTION_RESULT
    };

    static Clock::time_point after(double seconds)
    {
        if (!std::isfinite(seconds) || seconds < 0.0 || seconds > MAX_DURATION_SEC) {
            throw std::invalid_argument("duration must be in [0, 86400] seconds");
        }
        return Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(seconds));
    }

    void transition(Phase phase, const std::string &name, double timeout)
    {
        phase_ = phase;
        phase_name_ = name;
        deadline_ = after(timeout);
        auto feedback = std::make_shared<ExecuteSequence::Feedback>();
        feedback->step_index = static_cast<uint32_t>(index_);
        feedback->phase = name;
        goal_->publish_feedback(feedback);
    }

    void start(const std::shared_ptr<SequenceGoal> &goal)
    {
        goal_ = goal;
        index_ = 0;
        stopping_ = false;
        stop_message_.clear();
        cancel_sent_ = false;
        sequence_deadline_ = after(sequence_timeout_);
        try {
            // Compile the entire action before sending any hardware command.
            auto candidate = debug_ ? SequenceConfig::load(config_file_) : *config_;
            const auto request = goal_->get_goal();
            const bool lifecycle = request->kind == ExecuteSequence::Goal::START ||
                request->kind == ExecuteSequence::Goal::INITIALIZE ||
                request->kind == ExecuteSequence::Goal::END;
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
        stopping_ = true;
        abandon_suction_check();
        stop_message_ = message;
        stop_deadline_ = after(stop_timeout_);
        RCLCPP_WARN(get_logger(), "Stopping sequence: %s", message.c_str());
    }

    void finish(bool success, const std::string &message)
    {
        abandon_suction_check();
        auto result = std::make_shared<ExecuteSequence::Result>();
        result->success = success;
        result->message = message;
        if (goal_->is_canceling()) {
            result->success = false;
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
                 const std::shared_ptr<typename Service::Request> &value, Callback callback)
    {
        service_pending_ = true;
        auto pending = client->async_send_request(value,
            [this, callback](typename rclcpp::Client<Service>::SharedFuture response) {
                service_pending_ = false;
                abandon_request_ = {};
                if (interrupted()) {
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
        } else if (Clock::now() >= sequence_deadline_) {
            begin_stop("sequence timeout");
        }
        if (stopping_) {
            if (route_goal_ && !cancel_sent_) {
                cancel_sent_ = true;
                follower_->async_cancel_goal(route_goal_);
            }
            if (!service_pending_ && !route_pending_ && !route_goal_) {
                finish(false, stop_message_);
            } else if (Clock::now() >= stop_deadline_) {
                // A lost reply can conceal an active command; require a restart.
                faulted_ = true;
                if (abandon_request_) {
                    abandon_request_();
                    abandon_request_ = {};
                }
                finish(false, stop_message_ + "; stop unconfirmed, sequencer restart required");
            }
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
            if (suction_active_) {
                throw std::logic_error("sequence ended before waiting for its suction check");
            }
            finish(true, "sequence completed");
            return;
        }
        const auto &step = steps_[index_];
        switch (step.type) {
        case StepType::MOVE:
            generate_request_ = std::make_shared<GenerateRoute::Request>();
            // Reach each waypoint before advancing to the next sequence step.
            {
                const auto &target = step.pose;
                generate_request_->x = target[0];
                generate_request_->y = target[1];
                generate_request_->z = target[2];
                generate_request_->phi = target[3];
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

    void follow()
    {
        FollowRoute::Goal goal;
        goal.start = true;
        goal.path = planned_path_;
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
                transition(Phase::FOLLOWING, "following route", route_timeout_);
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
                ++index_;
                transition(Phase::READY, "ready", service_timeout_);
            } catch (const std::exception &error) {
                begin_stop(std::string("route result: ") + error.what());
            }
        };
        follower_->async_send_goal(goal, options);
    }

    std::string config_file_, team_, phase_name_, stop_message_;
    bool debug_{false}, stopping_{false}, faulted_{false};
    bool service_pending_{false}, route_pending_{false}, cancel_sent_{false};
    bool suction_pending_{false}, suction_active_{false};
    uint64_t suction_generation_{0};
    double service_timeout_, route_timeout_, sequence_timeout_, stop_timeout_;
    Phase phase_{Phase::READY};
    Clock::time_point deadline_, sequence_deadline_, stop_deadline_;
    Clock::time_point suction_deadline_;
    std::unique_ptr<SequenceConfig> config_;
    std::vector<Step> steps_;
    size_t index_{0};
    nav_msgs::msg::Path planned_path_;
    GenerateRoute::Request::SharedPtr generate_request_;
    PumpControl::Request::SharedPtr pump_request_;
    CheckSuction::Response::SharedPtr suction_reply_;
    std::function<void()> abandon_request_;
    std::function<void()> abandon_suction_;
    std::shared_ptr<SequenceGoal> goal_;
    RouteGoal::SharedPtr route_goal_;
    rclcpp::Client<Trigger>::SharedPtr initialization_;
    rclcpp::Client<GenerateRoute>::SharedPtr planner_;
    rclcpp::Client<PumpControl>::SharedPtr pump_writer_;
    rclcpp::Client<CheckSuction>::SharedPtr suction_checker_;
    rclcpp::Client<EndeffectorControl>::SharedPtr endeffector_;
    rclcpp_action::Client<FollowRoute>::SharedPtr follower_;
    rclcpp_action::Server<ExecuteSequence>::SharedPtr server_;
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
