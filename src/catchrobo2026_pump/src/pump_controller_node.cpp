#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_msgs/srv/check_suction.hpp"
#include "catchrobo2026_pump/pump_command.hpp"

using namespace std::chrono_literals;
using PumpControl = catchrobo2026_msgs::srv::PumpControl;
using CheckSuction = catchrobo2026_msgs::srv::CheckSuction;

class PumpControllerNode : public rclcpp::Node {
public:
    PumpControllerNode() : Node("pump_controller_node") {
        // Configure wiring and polarity at startup, in L/C/R order.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        pump_bits_ = this->declare_parameter<std::vector<int64_t>>(
            "pump_bits", {0, 1, 2}, descriptor);
        valve_bits_ = this->declare_parameter<std::vector<int64_t>>(
            "valve_bits", {3, 4, 5}, descriptor);
        pump_on_level_ = this->declare_parameter<bool>("pump_on_level", false, descriptor);
        valve_on_level_ = this->declare_parameter<bool>("valve_on_level", false, descriptor);
        const auto initial_state = this->declare_parameter<int64_t>(
            "initial_state", PumpControl::Request::SUCTION, descriptor);
        if (initial_state < PumpControl::Request::RELEASE ||
            initial_state > PumpControl::Request::SUCTION) {
            throw std::invalid_argument("initial_state must be -1, 0 or 1");
        }
        collector_states_.fill(static_cast<int8_t>(initial_state));

        if (pump_bits_.size() != 3 || valve_bits_.size() != 3) {
            throw std::invalid_argument("pump_bits and valve_bits must each contain 3 entries (L/C/R)");
        }
        auto bits = pump_bits_;
        bits.insert(bits.end(), valve_bits_.begin(), valve_bits_.end());
        std::array<bool, 6> used_bits{};
        for (const auto bit : bits) {
            if (bit < 0 || bit > 5 || used_bits[bit]) {
                throw std::invalid_argument("pump_bits and valve_bits must use bits 0..5 without duplicates");
            }
            used_bits[bit] = true;
        }

        pressure_indices_ = this->declare_parameter<std::vector<int64_t>>(
            "pressure_indices", {0, 1, 2}, descriptor);
        if (pressure_indices_.size() != 3) {
            throw std::invalid_argument("pressure_indices must contain 3 entries (L/C/R)");
        }
        std::array<bool, 3> used_indices{};
        for (const auto index : pressure_indices_) {
            if (index < 0 || index > 2 || used_indices[index]) {
                throw std::invalid_argument("pressure_indices must use indices 0..2 without duplicates");
            }
            used_indices[index] = true;
        }
        const auto threshold = this->declare_parameter<int64_t>(
            "pressure_threshold", 0x60, descriptor);
        if (threshold < std::numeric_limits<int32_t>::min() ||
            threshold > std::numeric_limits<int32_t>::max()) {
            throw std::invalid_argument("pressure_threshold must fit int32");
        }
        pressure_threshold_ = static_cast<int32_t>(threshold);
        pressure_comparison_ = this->declare_parameter<std::string>(
            "pressure_comparison", "unconfigured", descriptor);
        if (pressure_comparison_ != "ge" && pressure_comparison_ != "le" &&
            pressure_comparison_ != "unconfigured") {
            throw std::invalid_argument("pressure_comparison must be ge, le or unconfigured");
        }
        const auto max_pending = this->declare_parameter<int64_t>(
            "max_pending_suction_checks", 8, descriptor);
        if (max_pending < 1 || max_pending > 128) {
            throw std::invalid_argument("max_pending_suction_checks must be in 1..128");
        }
        max_pending_suction_checks_ = static_cast<size_t>(max_pending);
        pending_suction_checks_.reserve(max_pending_suction_checks_);

        pump_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("pump_state", 10);
        service_server_ = this->create_service<PumpControl>(
            "set_pump_state",
            std::bind(&PumpControllerNode::handle_service, this,
                      std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(
            10ms, std::bind(&PumpControllerNode::timer_callback, this));

        // Defer responses without blocking the executor or creating worker threads.
        suction_service_ = this->create_service<CheckSuction>(
            "check_suction",
            [this](const std::shared_ptr<rmw_request_id_t> header,
                   const std::shared_ptr<CheckSuction::Request> request) {
                handle_suction_request(header, request);
            });
        pressure_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "pressure_sensor", rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&PumpControllerNode::pressure_callback, this, std::placeholders::_1));
        suction_timer_ = this->create_wall_timer(
            10ms, [this]() { expire_suction_checks(SteadyClock::now()); });

        RCLCPP_INFO(this->get_logger(), "Pump Controller Node started.");
        if (pressure_comparison_ == "unconfigured") {
            RCLCPP_WARN(this->get_logger(),
                        "check_suction is unavailable until pressure_comparison is set to ge or le.");
        }
    }

private:
    using SteadyClock = std::chrono::steady_clock;

    struct PendingSuctionCheck {
        std::shared_ptr<rmw_request_id_t> header;
        uint8_t collector_mask;
        SteadyClock::time_point deadline;
        CheckSuction::Response response;
        bool received_sample = false;
    };

    void send_suction_response(
        const std::shared_ptr<rmw_request_id_t> &header, CheckSuction::Response &response)
    {
        try {
            suction_service_->send_response(*header, response);
        } catch (const std::exception &error) {
            RCLCPP_WARN(this->get_logger(), "Failed to send check_suction response: %s", error.what());
        }
    }

    void handle_suction_request(
        const std::shared_ptr<rmw_request_id_t> &header,
        const std::shared_ptr<CheckSuction::Request> &request)
    {
        const auto now = SteadyClock::now();
        expire_suction_checks(now);
        CheckSuction::Response response;
        response.pressure.fill(std::numeric_limits<int32_t>::min());
        if (request->collector_mask == 0 || (request->collector_mask & 0xf8) != 0) {
            response.message = "collector_mask must be in 1..7 (L/C/R bits 0/1/2)";
        } else if (!std::isfinite(request->timeout_sec) || request->timeout_sec <= 0.0 ||
                   request->timeout_sec > 86400.0) {
            response.message = "timeout_sec must be finite and in (0, 86400]";
        } else if (pressure_comparison_ == "unconfigured") {
            response.message = "pressure_comparison must be configured as ge or le";
        } else if (pending_suction_checks_.size() >= max_pending_suction_checks_) {
            response.message = "too many pending check_suction requests";
        } else {
            const auto duration = std::chrono::duration_cast<SteadyClock::duration>(
                std::chrono::duration<double>(request->timeout_sec));
            pending_suction_checks_.push_back(
                {header, request->collector_mask, now + duration, response, false});
            return;
        }
        send_suction_response(header, response);
    }

    void expire_suction_checks(SteadyClock::time_point now) {
        for (auto it = pending_suction_checks_.begin(); it != pending_suction_checks_.end();) {
            if (now < it->deadline) {
                ++it;
                continue;
            }
            it->response.timed_out = true;
            it->response.message = it->received_sample ?
                "suction threshold was not reached before timeout" :
                "no valid pressure sample received before timeout";
            send_suction_response(it->header, it->response);
            it = pending_suction_checks_.erase(it);
        }
    }

    void pressure_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        const auto now = SteadyClock::now();
        // Expire first so a late sample cannot turn a timeout into success.
        expire_suction_checks(now);
        const auto &dimensions = msg->layout.dim;
        if (msg->data.size() != 3 || msg->layout.data_offset != 0 ||
            (!dimensions.empty() && (dimensions.size() != 1 ||
             dimensions[0].size != 3 || dimensions[0].stride != 3))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring malformed pressure_sensor array (expected 3 values, offset 0).");
            return;
        }
        std::array<int32_t, 3> pressure;
        uint8_t suction_mask = 0;
        for (size_t i = 0; i < pressure.size(); ++i) {
            pressure[i] = msg->data[pressure_indices_[i]];
            if ((pressure_comparison_ == "ge" && pressure[i] >= pressure_threshold_) ||
                (pressure_comparison_ == "le" && pressure[i] <= pressure_threshold_)) {
                suction_mask |= static_cast<uint8_t>(1u << i);
            }
        }
        for (auto it = pending_suction_checks_.begin(); it != pending_suction_checks_.end();) {
            // Each request only observes callbacks after its acceptance.
            it->received_sample = true;
            it->response.pressure = pressure;
            it->response.suction_mask = suction_mask;
            if ((suction_mask & it->collector_mask) != it->collector_mask) {
                ++it;
                continue;
            }
            it->response.success = true;
            it->response.message = "all selected collectors reached the suction threshold";
            send_suction_response(it->header, it->response);
            it = pending_suction_checks_.erase(it);
        }
    }

    void handle_service(
        const std::shared_ptr<PumpControl::Request> request,
        std::shared_ptr<PumpControl::Response> response)
    {
        const std::array<int8_t, 3> states = {request->left, request->center, request->right};
        // Validate every state before updating any collector.
        for (const auto state : states) {
            if (state < PumpControl::Request::RELEASE || state > PumpControl::Request::SUCTION) {
                response->success = false;
                RCLCPP_WARN(this->get_logger(), "left, center and right must each be -1, 0 or 1.");
                return;
            }
        }
        collector_states_ = states;
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Collectors: L=%d, C=%d, R=%d",
                    request->left, request->center, request->right);
    }

    void timer_callback() {
        // Map logical states to CAN bit levels; the MCU inverts them at the GPIO.
        std::array<bool, 6> levels{};
        for (size_t i = 0; i < collector_states_.size(); ++i) {
            // Suction enables the pump; release enables the valve.
            const bool pump_on = collector_states_[i] == PumpControl::Request::SUCTION;
            const bool valve_on = collector_states_[i] == PumpControl::Request::RELEASE;
            levels[pump_bits_[i]] = pump_on == pump_on_level_;
            levels[valve_bits_[i]] = valve_on == valve_on_level_;
        }

        PumpCommand command{};
        command.bits.pump1 = levels[0];
        command.bits.pump2 = levels[1];
        command.bits.pump3 = levels[2];
        command.bits.valve1 = levels[3];
        command.bits.valve2 = levels[4];
        command.bits.valve3 = levels[5];

        // Publish a host-order integer; the CAN bridge applies htobe32.
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {static_cast<int32_t>(command.raw)};
        pump_pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pump_pub_;
    rclcpp::Service<PumpControl>::SharedPtr service_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<CheckSuction>::SharedPtr suction_service_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pressure_sub_;
    rclcpp::TimerBase::SharedPtr suction_timer_;

    std::vector<int64_t> pressure_indices_;
    int32_t pressure_threshold_;
    std::string pressure_comparison_;
    size_t max_pending_suction_checks_;
    std::vector<PendingSuctionCheck> pending_suction_checks_;

    std::array<int8_t, 3> collector_states_;
    std::vector<int64_t> pump_bits_;
    std::vector<int64_t> valve_bits_;
    bool pump_on_level_;
    bool valve_on_level_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PumpControllerNode>());
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("pump_controller_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
