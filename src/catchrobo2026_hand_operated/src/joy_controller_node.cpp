#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ros2_inverse_kinematics/robot_kinematics.h"

// サービス型のインクルード
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_msgs/srv/endeffector_control.hpp"

using namespace std::chrono_literals;

class JoyControllerNode : public rclcpp::Node {
public:
    JoyControllerNode() : Node("joy_controller_node") {
        
        // 1. パブリッシャー (各モーターの4つの角度を出力)
        joint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);

        // 2. サブスクライバー
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&JoyControllerNode::pose_callback, this, std::placeholders::_1));
            
        pump_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "pump_state", 10, std::bind(&JoyControllerNode::pump_callback, this, std::placeholders::_1));
            
        // 【追加】Endeffector状態のサブスクライバー
        endeffector_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "endeffector_state", 10, std::bind(&JoyControllerNode::endeffector_callback, this, std::placeholders::_1));

        // 3. サービスクライアント
        pump_client_ = this->create_client<catchrobo2026_msgs::srv::PumpControl>("set_pump_state");
        // 【追加】Endeffector切り替え用サービスクライアント
        endeffector_client_ = this->create_client<catchrobo2026_msgs::srv::EndeffectorControl>("set_endeffector_state");

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

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started with integrated IK, Pump, and Endeffector control.");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 自動制御ノード等からtarget_poseを受け取った場合、基準座標を上書きする
        // 単位を [m] から [mm] に変換
        current_pose_[0] = msg->pose.position.x * 1000.0;
        current_pose_[1] = msg->pose.position.y * 1000.0;
        current_pose_[2] = msg->pose.position.z * 1000.0;

        // クォータニオンからRPYを取得して角度を上書き
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        current_pose_[3] = static_cast<float>(yaw);   // PHI
        current_pose_[4] = static_cast<float>(pitch); // THE
        current_pose_[5] = static_cast<float>(roll);  // PSI
    }

    void pump_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            current_pump_val_ = msg->data[0];
        }
    }

    // 【追加】Endeffector状態のコールバック
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

        // --- 〇ボタン(buttons[1])によるPump状態の遷移 ---
        bool current_o_button = msg->buttons[1]; 

        if (current_o_button && !prev_o_button_) {
            if (!pump_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Pump service not ready.");
            } else {
                auto request = std::make_shared<catchrobo2026_msgs::srv::PumpControl::Request>();
                
                // 現在の値から次のコマンドを決定 (1:56, 2:0, 3:7)
                if (current_pump_val_ == 63) {
                    request->command = 2; // 次は 0
                } else if (current_pump_val_ == 0) {
                    request->command = 3; // 次は 7
                } else {
                    request->command = 1; // それ以外(7など)なら 56 に戻す
                }

                pump_client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Requested Pump change. Sent command: %d", request->command);
            }
        }
        prev_o_button_ = current_o_button;

        // --- 【追加】ボタン(例: buttons[2])によるEndeffector状態の遷移 ---
        bool current_endeffector_button = msg->buttons[2]; // 任意のボタンに変更可能

        if (current_endeffector_button && !prev_endeffector_button_) {
            if (!endeffector_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Endeffector service not ready.");
            } else {
                auto request = std::make_shared<catchrobo2026_msgs::srv::EndeffectorControl::Request>();
                
                // 現在の値から次のコマンドを決定 (endeffector_state_nodeの仕様に準拠: 0または1)
                if (current_endeffector_val_ == 0) {
                    request->command = 1; 
                } else {
                    request->command = 0; // 初期値の56や1の場合は0へ
                }

                endeffector_client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Requested Endeffector change. Sent command: %d", request->command);
            }
        }
        prev_endeffector_button_ = current_endeffector_button;
    }

    void publish_timer_callback() {
        // 1. Joy入力による手動介入 (位置の微調整)
        const float pos_gain = 5.0f;  
        const float rot_gain = 0.05f; 

        current_pose_[0] += vel_x_ * pos_gain;
        current_pose_[1] += vel_y_ * pos_gain;
        current_pose_[2] += vel_z_ * pos_gain;
        current_pose_[3] += vel_phi_ * rot_gain;
        // 必要な場合は Theta, Psi の速度も追加加算してください

        // 2. ローカルで逆運動学(IK)を計算
        float target_joints[4] = {0.0f};
        kin_.inverse_kinematics(current_pose_, target_joints);

        // 3. 計算結果をパブリッシュ
        std_msgs::msg::Float32MultiArray msg_out;
        msg_out.data.resize(4);
        for(int i = 0; i < 4; ++i) {
            msg_out.data[i] = target_joints[i];
        }
        joint_pub_->publish(msg_out);
    }

    // --- 変数定義 ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pump_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr endeffector_sub_; // 【追加】
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
    rclcpp::Client<catchrobo2026_msgs::srv::PumpControl>::SharedPtr pump_client_;
    rclcpp::Client<catchrobo2026_msgs::srv::EndeffectorControl>::SharedPtr endeffector_client_; // 【追加】
    
    rclcpp::TimerBase::SharedPtr publish_timer_; 
    
    robot_kinematics kin_; // 運動学クラスのインスタンス

    float current_pose_[6];
    int current_pump_val_ = 56;
    int current_endeffector_val_ = 56; // 【追加】初期値はノード側に合わせる
    bool prev_o_button_ = false; 
    bool prev_endeffector_button_ = false; // 【追加】
    
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