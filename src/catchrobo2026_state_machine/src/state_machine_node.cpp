#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
// 新しいサービス型のヘッダーをインクルード
#include "catchrobo2026_msgs/srv/state_control.hpp"

using namespace std::chrono_literals;

class RepeaterNode : public rclcpp::Node
{
public:
  RepeaterNode()
  : Node("repeater_node"), current_value_(0)
  {
    // パブリッシャーの設定 (トピック名: 'repeated_value')
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("init_state", 10);
    
    // StateControl サービスを使用するようにサーバーを設定
    service_ = this->create_service<catchrobo2026_msgs::srv::StateControl>(
      "set_value",
      std::bind(&RepeaterNode::service_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    // タイマーの設定 (100ms = 10Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&RepeaterNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "C++ Repeater Node が初期化されました");
  }

private:
  void service_callback(
    const std::shared_ptr<catchrobo2026_msgs::srv::StateControl::Request> request,
    std::shared_ptr<catchrobo2026_msgs::srv::StateControl::Response> response)
  {
    // リクエストの command を受け取る[cite: 4]
    current_value_ = request->command;
    RCLCPP_INFO(this->get_logger(), "新しい値を受け取りました: %d", current_value_);
    
    // レスポンスの success に true を返す[cite: 4]
    response->success = true;
  }

  void timer_callback()
  {
    auto msg = std_msgs::msg::Int32();
    msg.data = current_value_;
    publisher_->publish(msg);
  }

  int current_value_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Service<catchrobo2026_msgs::srv::StateControl>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepeaterNode>());
  rclcpp::shutdown();
  return 0;
}