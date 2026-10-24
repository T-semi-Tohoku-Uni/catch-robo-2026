#include <algorithm>
#include <array>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_yaml_param_parser/parser.h"
#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// 追加: マーカー用インクルード
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ros2_inverse_kinematics/robot_kinematics.h"

// サービス型のインクルード
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace {
constexpr auto kPublishPeriod = 2ms;
constexpr float kPublishPeriodSeconds =
    std::chrono::duration<float>(kPublishPeriod).count();
constexpr auto kManualVelocityReloadPeriod = 500ms;
constexpr double kDefaultManualLinearSpeedMmS = 50.0;
}

class JoyControllerNode : public rclcpp::Node {
public:
    JoyControllerNode() : Node("joy_controller_node") {
        
        // Publish the three physical joint angles.
        joint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);
        
        // 追加: マーカーパブリッシャー
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("target_arm_markers", 10);

        // 2. サブスクライバー
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&JoyControllerNode::pose_callback, this, std::placeholders::_1));

        // 3. サービスクライアント
        pump_client_ = this->create_client<catchrobo2026_msgs::srv::PumpControl>("set_pump_state");
        state_client_ = this->create_client<std_srvs::srv::Trigger>("request_initialization");

        rcl_interfaces::msg::ParameterDescriptor velocity_descriptor;
        velocity_descriptor.read_only = true;
        velocity_descriptor.description = "Maximum manual velocity at full Joy input";
        manual_linear_speed_mm_s_ = declare_parameter(
            "manual_linear_speed_mm_s", kDefaultManualLinearSpeedMmS, velocity_descriptor);
        selected_pump_ = declare_parameter("selected_pump", 1, velocity_descriptor);
        if (selected_pump_ < 1 || selected_pump_ > 3) {
            throw std::invalid_argument("selected_pump must be 1, 2 or 3");
        }
        if (!valid_manual_rate(manual_linear_speed_mm_s_)) {
            throw std::invalid_argument("manual_linear_speed_mm_s must be finite and nonnegative");
        }

        rcl_interfaces::msg::ParameterDescriptor reload_descriptor;
        reload_descriptor.read_only = true;
        reload_descriptor.description = "Reload manual velocity YAML while debug mode is enabled";
        debug_ = declare_parameter("debug", false, reload_descriptor);
        manual_velocity_config_ = declare_parameter(
            "manual_velocity_config", std::string{}, reload_descriptor);
        if (debug_) {
            reload_manual_velocity_config();
            manual_velocity_reload_timer_ = create_wall_timer(
                kManualVelocityReloadPeriod,
                std::bind(&JoyControllerNode::manual_velocity_reload_timer_callback, this));
        }
        // Keep the command period explicit so configured speeds have stable units.
        publish_timer_ = this->create_wall_timer(
            kPublishPeriod, std::bind(&JoyControllerNode::publish_timer_callback, this));

        // 目標座標の初期値設定 [mm] および [rad]
        current_pose_[0] = 670.0f;  // X
        current_pose_[1] = -110.0f;  // Y
        current_pose_[2] = 220.0f;  // Z
        current_pose_[3] = 0.0f;    // Phi (Yaw相当)
        current_pose_[4] = -M_PI / 2.0f; // Theta (Pitch相当、デフォルト姿勢)
        current_pose_[5] = 0.0f;    // Psi (Roll相当)

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started with integrated IK, Pump control and Markers.");
    }

private:
    using PumpRequest = catchrobo2026_msgs::srv::PumpControl::Request;

    static bool valid_manual_rate(double value) {
        return std::isfinite(value) && value >= 0.0 &&
            value <= static_cast<double>(std::numeric_limits<float>::max());
    }

    static std::string take_rcutils_error(const std::string &fallback) {
        const auto error = rcutils_get_error_string();
        const std::string message = error.str[0] == '\0' ? fallback : error.str;
        rcutils_reset_error();
        return message;
    }

    bool reload_manual_velocity_config() {
        try {
            if (manual_velocity_config_.empty()) {
                throw std::invalid_argument("manual_velocity_config is empty");
            }
            const std::string node_fqn = get_fully_qualified_name();
            std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> yaml_params(
                rcl_yaml_node_struct_init(rcutils_get_default_allocator()),
                rcl_yaml_node_struct_fini);
            if (!yaml_params) {
                throw std::runtime_error(take_rcutils_error(
                    "failed to allocate the YAML parameter structure"));
            }
            if (!rcl_parse_yaml_file(manual_velocity_config_.c_str(), yaml_params.get())) {
                throw std::runtime_error(take_rcutils_error(
                    "failed to parse the manual velocity config"));
            }
            const auto parameter_map = rclcpp::parameter_map_from(
                yaml_params.get(), node_fqn.c_str());
            const auto node_parameters = parameter_map.find(node_fqn);
            if (node_parameters == parameter_map.end()) {
                throw std::invalid_argument(
                    "manual velocity config has no section matching " + node_fqn);
            }

            double linear_speed = kDefaultManualLinearSpeedMmS;
            for (const auto &parameter : node_parameters->second) {
                if (parameter.get_name() == "manual_linear_speed_mm_s") {
                    linear_speed = parameter.as_double();
                } else if (parameter.get_name().rfind("manual_", 0) == 0 &&
                    parameter.get_name() != "manual_velocity_config") {
                    throw std::invalid_argument(
                        "unknown manual velocity parameter: " + parameter.get_name());
                }
            }
            if (!valid_manual_rate(linear_speed)) {
                throw std::invalid_argument(
                    "manual_linear_speed_mm_s must be finite and nonnegative");
            }
            const bool changed = linear_speed != manual_linear_speed_mm_s_;
            const bool recovered = !manual_velocity_config_valid_;
            manual_linear_speed_mm_s_ = linear_speed;
            manual_velocity_config_valid_ = true;
            last_manual_velocity_error_.clear();
            if (changed || recovered) {
                RCLCPP_INFO(get_logger(),
                    "Loaded manual velocity config: linear=%.6g mm/s", linear_speed);
            }
            return true;
        } catch (const std::exception &error) {
            manual_velocity_config_valid_ = false;
            if (last_manual_velocity_error_ != error.what()) {
                RCLCPP_ERROR(get_logger(), "Manual velocity config is invalid: %s", error.what());
                last_manual_velocity_error_ = error.what();
            }
            return false;
        }
    }

    void manual_velocity_reload_timer_callback() {
        if (manual_input_neutral_ && !manual_input_active_ &&
            !manual_input_blocked_until_neutral_) {
            reload_manual_velocity_config();
        }
    }

    void clear_velocity() {
        vel_x_ = vel_y_ = vel_z_ = 0.0f;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto &position = msg->pose.position;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) return;
        current_pose_[0] = position.x * 1000.0f;
        current_pose_[1] = position.y * 1000.0f;
        current_pose_[2] = position.z * 1000.0f;
        current_pose_[3] = 0.0f;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 6 || msg->buttons.size() < 6) {
            clear_velocity();
            manual_input_active_ = false;
            manual_input_neutral_ = false;
            manual_input_blocked_until_neutral_ = true;
            return;
        }
        
        // --- 速度として入力値を保持 ---
        {
            // 左スティック・十字キーともに横を X、縦を Y に割り当てる。
            // 十字キーの軸がないコントローラーではスティックのみ使用する。
            const float dpad_x = msg->axes.size() > 6 ? msg->axes[6] : 0.0f;
            const float dpad_y = msg->axes.size() > 7 ? msg->axes[7] : 0.0f;
            const float next_vel_x = std::clamp(msg->axes[0] + dpad_x, -1.0f, 1.0f);
            const float next_vel_y = std::clamp(msg->axes[1] + dpad_y, -1.0f, 1.0f);
            const float next_vel_z = msg->axes[4];
            const bool button_input = std::any_of(
                msg->buttons.begin(), msg->buttons.end(),
                [](int32_t value) { return value != 0; });
            const bool finite_input = std::isfinite(next_vel_x) &&
                std::isfinite(next_vel_y) && std::isfinite(next_vel_z);
            const bool neutral_input = finite_input && next_vel_x == 0.0f &&
                next_vel_y == 0.0f && next_vel_z == 0.0f &&
                !button_input;
            manual_input_neutral_ = neutral_input;

            if (neutral_input) {
                manual_input_active_ = false;
                manual_input_blocked_until_neutral_ = false;
                clear_velocity();
            } else if (!finite_input) {
                manual_input_active_ = false;
                manual_input_blocked_until_neutral_ = true;
                clear_velocity();
            } else if (manual_input_blocked_until_neutral_) {
                clear_velocity();
            } else {
                if (!manual_input_active_ && debug_ && !reload_manual_velocity_config()) {
                    manual_input_blocked_until_neutral_ = true;
                    clear_velocity();
                } else {
                    vel_x_ = next_vel_x;
                    vel_y_ = next_vel_y;
                    vel_z_ = next_vel_z;
                    manual_input_active_ = true;
                }
            }
        }

        if (manual_input_blocked_until_neutral_) {
            prev_x_button_ = msg->buttons[0] != 0;
            prev_o_button_ = msg->buttons[1] != 0;
            return;
        }

        // Request a shared initialization toggle on each X button press.
        const bool current_x_button = msg->buttons[0] != 0;
        if (current_x_button && !prev_x_button_ && !state_request_pending_) {
            if (!state_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "State service not ready.");
            } else {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                state_request_pending_ = true;
                state_request_time_ = std::chrono::steady_clock::now();
                state_request_id_ = state_client_->async_send_request(request,
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                        state_request_pending_ = false;
                        if (!future.get()->success) {
                            RCLCPP_WARN(this->get_logger(), "State request was rejected.");
                            return;
                        }
                        RCLCPP_INFO(this->get_logger(), "Initialization request accepted.");
                    }).request_id;
            }
        }
        prev_x_button_ = current_x_button;

        // --- 〇ボタン(buttons[1])によるPump状態の遷移 ---
        bool current_o_button = msg->buttons[1]; 

        if (current_o_button && !prev_o_button_ && !pump_request_pending_) {
            if (!pump_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Pump service not ready.");
            } else {
                auto request = std::make_shared<PumpRequest>();
                request->left = selected_pump_ == 1 ? next_pump_state_ : PumpRequest::OFF;
                request->center = selected_pump_ == 2 ? next_pump_state_ : PumpRequest::OFF;
                request->right = selected_pump_ == 3 ? next_pump_state_ : PumpRequest::OFF;
                pump_request_pending_ = true;
                pump_request_time_ = std::chrono::steady_clock::now();
                pump_request_id_ = pump_client_->async_send_request(request,
                    [this](rclcpp::Client<catchrobo2026_msgs::srv::PumpControl>::SharedFuture future) {
                        pump_request_pending_ = false;
                        if (!future.get()->success) {
                            RCLCPP_WARN(this->get_logger(), "Pump request was rejected.");
                            return;
                        }
                        if (next_pump_state_ == PumpRequest::OFF) {
                            next_pump_state_ = PumpRequest::SUCTION;
                        } else if (next_pump_state_ == PumpRequest::SUCTION) {
                            next_pump_state_ = PumpRequest::RELEASE;
                        } else {
                            next_pump_state_ = PumpRequest::OFF;
                        }
                    }).request_id;
            }
        }
        prev_o_button_ = current_o_button;

    }

    void publish_timer_callback() {
        if (state_request_pending_ &&
            std::chrono::steady_clock::now() - state_request_time_ >= 1s) {
            state_client_->remove_pending_request(state_request_id_);
            state_request_pending_ = false;
            RCLCPP_WARN(this->get_logger(), "State request timed out. Press again to retry.");
        }

        // 応答が途絶えても、次のボタン操作で同じ状態を再要求できるようにする。
        if (pump_request_pending_ &&
            std::chrono::steady_clock::now() - pump_request_time_ >= 1s) {
            pump_client_->remove_pending_request(pump_request_id_);
            pump_request_pending_ = false;
            RCLCPP_WARN(this->get_logger(), "Pump request timed out. Press again to retry.");
        }

        // 1. Joy入力による手動介入 (位置の微調整)
        const float pos_gain = static_cast<float>(manual_linear_speed_mm_s_) *
            kPublishPeriodSeconds;

        current_pose_[0] -= vel_x_ * pos_gain;
        current_pose_[1] += vel_y_ * pos_gain;
        current_pose_[2] += vel_z_ * pos_gain;

        // 2. ローカルで逆運動学(IK)を計算
        float target_joints[4] = {0.0f};
        kin_.inverse_kinematics(current_pose_, target_joints);

        target_joints[3] = 0.0f;  // Only the library boundary has a fourth joint.

        if (!std::all_of(target_joints, target_joints + 3,
                [](float value) { return std::isfinite(value); })) return;

        // Publish the three physical joint commands.
        std_msgs::msg::Float32MultiArray msg_out;
        msg_out.data.resize(3);
        for(int i = 0; i < 3; ++i) {
            msg_out.data[i] = target_joints[i];
        }
        joint_pub_->publish(msg_out);

        // 4. アームの姿勢マーカー (矢印) をパブリッシュ
        publish_arm_markers(target_joints);
    }

    void publish_arm_markers(float* target_joints) {
        float positions[6][3];
        // 運動学モデルから6つのジョイント位置(0~5)の座標配列を取得
        kin_.get_joint_positions(target_joints, positions);

        visualization_msgs::msg::MarkerArray marker_array;
        
        // 6つの点から5本の矢印(リンク)を生成する
        for (int i = 0; i < 5; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // 適切な固定フレーム名に変更してください (例: "base_link" や "map")
            marker.header.stamp = this->now();
            marker.ns = "arm_links";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p_start, p_end;
            // robot_kinematicsは[mm]単位なので、RViz用に[m]に変換する[cite: 1]
            p_start.x = positions[i][0] / 1000.0;
            p_start.y = positions[i][1] / 1000.0;
            p_start.z = positions[i][2] / 1000.0;

            p_end.x = positions[i+1][0] / 1000.0;
            p_end.y = positions[i+1][1] / 1000.0;
            p_end.z = positions[i+1][2] / 1000.0;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);

            // 矢印の太さ設定
            marker.scale.x = 0.015; // シャフトの直径 (1.5cm)
            marker.scale.y = 0.03;  // ヘッドの直径 (3.0cm)
            marker.scale.z = 0.03;  // ヘッドの長さ (3.0cm)

            // 色の設定 (例として緑色)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f; // 若干透過

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    // --- 変数定義 ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // 追加
    
    rclcpp::Client<catchrobo2026_msgs::srv::PumpControl>::SharedPtr pump_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr state_client_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_; 
    rclcpp::TimerBase::SharedPtr manual_velocity_reload_timer_;
    
    robot_kinematics kin_; // 運動学クラスのインスタンス

    float current_pose_[6];
    double manual_linear_speed_mm_s_ = kDefaultManualLinearSpeedMmS;
    bool debug_ = false;
    std::string manual_velocity_config_;
    bool manual_velocity_config_valid_ = true;
    bool manual_input_neutral_ = true;
    bool manual_input_active_ = false;
    bool manual_input_blocked_until_neutral_ = false;
    std::string last_manual_velocity_error_;
    bool prev_x_button_ = false;
    bool state_request_pending_ = false;
    int64_t state_request_id_ = 0;
    std::chrono::steady_clock::time_point state_request_time_;
    int selected_pump_{1};
    int8_t next_pump_state_ = PumpRequest::RELEASE;
    bool pump_request_pending_ = false;
    int64_t pump_request_id_ = 0;
    std::chrono::steady_clock::time_point pump_request_time_;
    bool prev_o_button_ = false; 
    
    float vel_x_ = 0.0f;
    float vel_y_ = 0.0f;
    float vel_z_ = 0.0f;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}
