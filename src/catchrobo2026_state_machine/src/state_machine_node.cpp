#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

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
    state_file_ = declare_parameter<std::string>("state_file", "");
    const bool require_state_file = declare_parameter("require_state_file", false);
    if (require_state_file && state_file_.empty()) {
      throw std::runtime_error("state_file is required");
    }
    if (!state_file_.empty()) {
      current_value_ = read_state_file(state_file_);
    }
    // パブリッシャーの設定 (トピック名: 'init_state')
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("init_state", 10);

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
        const int next_value = current_value_ == 0 ? 1 : 0;
        if (!store_state(next_value)) {
          response->success = false;
          response->message = "Failed to persist initialization state";
          return;
        }
        current_value_ = next_value;
        timer_callback();
        response->success = true;
        response->message = "Initialization requested";
        RCLCPP_INFO(this->get_logger(), "Initialization requested: %d", current_value_);
      });

    // タイマーの設定 (100ms = 10Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&RepeaterNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "C++ Repeater Node が初期化されました");
  }

private:
  static int read_state_file(const std::string & path)
  {
    std::ifstream input(path);
    int value;
    if (!(input >> value)) {
      throw std::runtime_error("Failed to read state_file: " + path);
    }
    input >> std::ws;
    if (!input.eof()) {
      throw std::runtime_error("Invalid state_file contents: " + path);
    }
    return value;
  }

  bool store_state(int value)
  {
    if (state_file_.empty()) {
      return true;
    }
    const auto path = std::filesystem::path(state_file_);
    const auto temporary = path.string() + ".tmp";
    try {
      {
        std::ofstream output(temporary, std::ios::trunc);
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output << value << '\n';
        output.close();
      }
      std::filesystem::rename(temporary, path);
      return true;
    } catch (const std::exception & error) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      RCLCPP_ERROR(get_logger(), "Failed to persist initialization state: %s", error.what());
      return false;
    }
  }

  void service_callback(
    const std::shared_ptr<catchrobo2026_msgs::srv::StateControl::Request> request,
    std::shared_ptr<catchrobo2026_msgs::srv::StateControl::Response> response)
  {
    // リクエストの command を受け取る[cite: 4]
    if (!store_state(request->command)) {
      response->success = false;
      return;
    }
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
  std::string state_file_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Service<catchrobo2026_msgs::srv::StateControl>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initialization_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepeaterNode>());
  rclcpp::shutdown();
  return 0;
}
