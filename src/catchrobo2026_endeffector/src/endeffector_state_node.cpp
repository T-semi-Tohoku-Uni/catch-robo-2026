#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "catchrobo2026_msgs/srv/endeffector_control.hpp"

using namespace std::chrono_literals;

class EndeffectorStateNode : public rclcpp::Node {
public:
    EndeffectorStateNode() : Node("endeffector_state_node"), current_endeffector_value_(56) {
        // main由来の起動値を維持する。56の実機での意味は未確認。
        endeffector_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("endeffector_state", 10);

        // 0または1を受け取り、現在の指令値を更新する。
        service_server_ = this->create_service<catchrobo2026_msgs::srv::EndeffectorControl>(
            "set_endeffector_state",
            std::bind(&EndeffectorStateNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 現在の指令値を50ms周期（20Hz）で配信する。
        timer_ = this->create_wall_timer(
            50ms, std::bind(&EndeffectorStateNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Endeffector State Node started.");
    }

private:
    void handle_service(
        const std::shared_ptr<catchrobo2026_msgs::srv::EndeffectorControl::Request> request,
        std::shared_ptr<catchrobo2026_msgs::srv::EndeffectorControl::Response> response)
    {
        int command = request->command;

        if (command == 0) {
            current_endeffector_value_ = 0;
            response->success = true;
        } else if (command == 1) {
            current_endeffector_value_ = 1;
            response->success = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid command received: %d. Expected 0 or 1.", command);
            response->success = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Endeffector state updated to command %d (Publishing value: %d)", command, current_endeffector_value_);
    }

    void timer_callback() {
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {current_endeffector_value_};
        endeffector_pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr endeffector_pub_;
    rclcpp::Service<catchrobo2026_msgs::srv::EndeffectorControl>::SharedPtr service_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    int current_endeffector_value_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndeffectorStateNode>());
    rclcpp::shutdown();
    return 0;
}
