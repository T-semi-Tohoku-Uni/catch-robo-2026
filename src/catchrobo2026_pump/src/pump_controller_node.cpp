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

using namespace std::chrono_literals;
using PumpControl = catchrobo2026_msgs::srv::PumpControl;

class PumpControllerNode : public rclcpp::Node {
public:
    PumpControllerNode() : Node("pump_controller_node") {
        // 配線・極性は起動時の設定。L/C/Rの順にCAN指令のビット番号を指定する。
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
        int32_t used_bits = 0;
        for (const auto bit : bits) {
            if (bit < 0 || bit > 5 || (used_bits & (1 << bit)) != 0) {
                throw std::invalid_argument("pump_bits and valve_bits must use bits 0..5 without duplicates");
            }
            used_bits |= 1 << bit;
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
        // 全値を確認してから更新し、不正な要求では一部だけ変更されないようにする。
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
        // CAN指令は6bit。GPIOのHigh/Lowへの変換はマイコン側で行う。
        int32_t state = 0;
        for (size_t i = 0; i < collector_states_.size(); ++i) {
            // 吸引はポンプON、開放は電磁弁ON、オフは両方OFF。
            const bool pump_on = collector_states_[i] == PumpControl::Request::SUCTION;
            const bool valve_on = collector_states_[i] == PumpControl::Request::RELEASE;
            if (pump_on == pump_on_level_) {
                state |= 1 << pump_bits_[i];
            }
            if (valve_on == valve_on_level_) {
                state |= 1 << valve_bits_[i];
            }
        }
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {state};
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
