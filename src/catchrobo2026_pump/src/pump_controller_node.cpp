#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_pump/pump_command.hpp"

using namespace std::chrono_literals;
using PumpControl = catchrobo2026_msgs::srv::PumpControl;

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

        pump_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("pump_state", 10);
        service_server_ = this->create_service<PumpControl>(
            "set_pump_state",
            std::bind(&PumpControllerNode::handle_service, this,
                      std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(
            10ms, std::bind(&PumpControllerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Pump Controller Node started.");
    }

private:
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

    std::array<int8_t, 3> collector_states_;
    std::vector<int64_t> pump_bits_;
    std::vector<int64_t> valve_bits_;
    bool pump_on_level_;
    bool valve_on_level_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PumpControllerNode>());
    rclcpp::shutdown();
    return 0;
}
