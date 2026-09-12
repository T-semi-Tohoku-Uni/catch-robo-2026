#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
// 新しいサービス型のヘッダーをインクルード
#include "catchrobo2026_msgs/srv/state_control.hpp"

using namespace std::chrono_literals;

class RepeaterNode : public rclcpp::Node
{
public:
  RepeaterNode()
  : Node("repeater_node"), current_value_(0)
  {
    // パブリッシャーの設定 (トピック名: 'init_state')
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("init_state", 10);
    endprocessing_publisher_ =
      this->create_publisher<std_msgs::msg::Int32MultiArray>("end_state", 10);
    
    // StateControl サービスを使用するようにサーバーを設定
    service_ = this->create_service<catchrobo2026_msgs::srv::StateControl>(
      "set_value",
      std::bind(&RepeaterNode::service_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    initialization_service_ = this->create_service<std_srvs::srv::Trigger>(
      "request_initialization",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Share the toggle state between UI and joystick requests.
        current_value_ = current_value_ == 0 ? 1 : 0;
        timer_callback();
        response->success = true;
        response->message = "Initialization requested";
        RCLCPP_INFO(this->get_logger(), "Initialization requested: %d", current_value_);
      });
    
    // タイマーの設定 (100ms = 10Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&RepeaterNode::timer_callback, this));
    endprocessing_timer_ = this->create_wall_timer(
      1ms, [this]() {
        if (endprocessing_remaining_ == 0) {
          return;
        }
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {1};
        endprocessing_publisher_->publish(msg);
        --endprocessing_remaining_;
      });
      
    RCLCPP_INFO(this->get_logger(), "C++ Repeater Node が初期化されました");
  }

private:
  void service_callback(
    const std::shared_ptr<catchrobo2026_msgs::srv::StateControl::Request> request,
    std::shared_ptr<catchrobo2026_msgs::srv::StateControl::Response> response)
  {
    if (request->command == 2) {
      // 再要求時は、その時点から20回の送信を開始する。
      endprocessing_remaining_ = 20;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "endprocessing に [1] を20回送信します");
      return;
    }

    // リクエストの command を受け取る[cite: 4]
    current_value_ = request->command;
    RCLCPP_INFO(this->get_logger(), "新しい値を受け取りました: %d", current_value_);
    
    // レスポンスの success に true を返す[cite: 4]
    response->success = true;
  }

  void timer_callback()
  {
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = {current_value_};
    publisher_->publish(msg);
  }

  int current_value_;
  int endprocessing_remaining_ = 0;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr endprocessing_publisher_;
  rclcpp::Service<catchrobo2026_msgs::srv::StateControl>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initialization_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr endprocessing_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepeaterNode>());
  rclcpp::shutdown();
  return 0;
}
