#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
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
#include "catchrobo2026_msgs/srv/endeffector_control.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class JoyControllerNode : public rclcpp::Node {
public:
    JoyControllerNode() : Node("joy_controller_node") {
        
        // 1. パブリッシャー (各モーターの4つの角度を出力)
        joint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);
        
        // 追加: マーカーパブリッシャー
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("target_arm_markers", 10);

        // 2. サブスクライバー
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&JoyControllerNode::pose_callback, this, std::placeholders::_1));
            
        endeffector_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "endeffector_state", 10, std::bind(&JoyControllerNode::endeffector_callback, this, std::placeholders::_1));

        // 3. サービスクライアント
        pump_client_ = this->create_client<catchrobo2026_msgs::srv::PumpControl>("set_pump_state");
        endeffector_client_ = this->create_client<catchrobo2026_msgs::srv::EndeffectorControl>("set_endeffector_state");
        state_client_ = this->create_client<std_srvs::srv::Trigger>("request_initialization");

        // 4. IK計算とパブリッシュを行うメインループタイマー (例: 20ms = 50Hz)
        publish_timer_ = this->create_wall_timer(
            20ms, std::bind(&JoyControllerNode::publish_timer_callback, this));

        // 目標座標の初期値設定 [mm] および [rad]
        current_pose_[0] = 600.0f;  // X
        current_pose_[1] = 200.0f;  // Y
        current_pose_[2] = 200.0f;  // Z
        current_pose_[3] = 0.0f;    // Phi (Yaw相当)
        current_pose_[4] = -M_PI / 2.0f; // Theta (Pitch相当、デフォルト姿勢)
        current_pose_[5] = 0.0f;    // Psi (Roll相当)

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started with integrated IK, Pump, Endeffector control, and Markers.");
    }

private:
    using PumpRequest = catchrobo2026_msgs::srv::PumpControl::Request;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 単位を [m] から [mm] に変換
        current_pose_[0] = msg->pose.position.x * 1000.0;
        current_pose_[1] = msg->pose.position.y * 1000.0;
        current_pose_[2] = msg->pose.position.z * 1000.0;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.orientation, q);

        // クォータニオンを回転行列に変換
        tf2::Matrix3x3 mat(q);

        // 【修正】Pitch = -90度の特異点（ジンバルロック）を回避するため、
        // 常に水平面上に残る「ローカルY軸」のベクトルを抽出してYawを計算します。
        tf2::Vector3 y_axis = mat.getColumn(1); 
        
        // Y軸ベクトルは [-sin(yaw), cos(yaw), 0]^T の形になるため、atan2でYawを逆算
        double yaw = std::atan2(-y_axis.x(), y_axis.y());

        current_pose_[3] = static_cast<float>(yaw);   // PHI
        
        // 4DoF IKでは THE(Pitch) と PSI(Roll) は使用しないため更新不要です
    }

    void endeffector_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            current_endeffector_val_ = msg->data[0];
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 6 || msg->buttons.size() < 6) return;
        
        // --- 速度として入力値を保持 ---
        vel_x_ = msg->axes[1]; 
        vel_y_ = msg->axes[0]; 
        vel_z_ = msg->axes[4]; 
        vel_phi_ = msg->axes[3]; 

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
                request->left = next_pump_state_;
                request->center = next_pump_state_;
                request->right = next_pump_state_;
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

        // --- buttons[2]によるEndeffector状態の切り替え ---
        bool current_endeffector_button = msg->buttons[2];

        if (current_endeffector_button && !prev_endeffector_button_) {
            if (!endeffector_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Endeffector service not ready.");
            } else {
                auto request = std::make_shared<catchrobo2026_msgs::srv::EndeffectorControl::Request>();

                if (current_endeffector_val_ == 0) {
                    request->command = 1;
                } else {
                    request->command = 0; 
                }

                endeffector_client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Requested Endeffector change. Sent command: %d", request->command);
            }
        }
        prev_endeffector_button_ = current_endeffector_button;
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
        const float pos_gain = 5.0f;  
        const float rot_gain = 0.05f; 

        current_pose_[0] += vel_x_ * pos_gain;
        current_pose_[1] += vel_y_ * pos_gain;
        current_pose_[2] += vel_z_ * pos_gain;
        current_pose_[3] += vel_phi_ * rot_gain;

        // 2. ローカルで逆運動学(IK)を計算
        float target_joints[4] = {0.0f};
        kin_.inverse_kinematics(current_pose_, target_joints);

        // 第4関節の目標角度を [-2π, 0] [rad] に制限する。
        target_joints[3] = std::max(-2.0f * static_cast<float>(M_PI),
                                    std::min(target_joints[3], 0.0f));

        // 3. 計算結果をパブリッシュ
        std_msgs::msg::Float32MultiArray msg_out;
        msg_out.data.resize(4);
        for(int i = 0; i < 4; ++i) {
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
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr endeffector_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // 追加
    
    rclcpp::Client<catchrobo2026_msgs::srv::PumpControl>::SharedPtr pump_client_;
    rclcpp::Client<catchrobo2026_msgs::srv::EndeffectorControl>::SharedPtr endeffector_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr state_client_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_; 
    
    robot_kinematics kin_; // 運動学クラスのインスタンス

    float current_pose_[6];
    bool prev_x_button_ = false;
    bool state_request_pending_ = false;
    int64_t state_request_id_ = 0;
    std::chrono::steady_clock::time_point state_request_time_;
    int8_t next_pump_state_ = PumpRequest::RELEASE;
    bool pump_request_pending_ = false;
    int64_t pump_request_id_ = 0;
    std::chrono::steady_clock::time_point pump_request_time_;
    int current_endeffector_val_ = 56; 
    bool prev_o_button_ = false; 
    bool prev_endeffector_button_ = false;
    
    float vel_x_ = 0.0f;
    float vel_y_ = 0.0f;
    float vel_z_ = 0.0f;
    float vel_phi_ = 0.0f;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}
