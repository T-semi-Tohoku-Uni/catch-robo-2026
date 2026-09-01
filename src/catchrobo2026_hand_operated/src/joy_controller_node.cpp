#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "catchrobo2026_msgs/srv/inverse_kinematics.hpp"

using namespace std::chrono_literals;

class JoyControllerNode : public rclcpp::Node {
public:
    JoyControllerNode() : Node("joy_controller_node") {
        
        // 1. パブリッシャー (各モーターの4つの角度を出力)
        joint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);

        // 2. サブスクライバー (Joy入力)
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        // 3. サービスクライアント (逆運動学)
        ik_client_ = this->create_client<catchrobo2026_msgs::srv::InverseKinematics>("inverse_kinematics");

        // 4. IK計算をリクエストするタイマー (例: 50ms = 20Hz)
        ik_timer_ = this->create_wall_timer(
            5ms, std::bind(&JoyControllerNode::ik_timer_callback, this));

        // 5. 【追加】一定周期でパブリッシュを行うタイマー (例: 20ms = 50Hz)
        publish_timer_ = this->create_wall_timer(
            10ms, std::bind(&JoyControllerNode::publish_timer_callback, this));

        // 目標座標の初期値
        target_pose_[0] = 600.0f;  // X
        target_pose_[1] = 200.0f; // Y
        target_pose_[2] = 200.0f;  // Z
        target_pose_[3] = 0.0f;    // Phi (Roll)
        target_pose_[4] = 0.0f;    // Theta (Pitch)
        target_pose_[5] = 0.0f;    // Psi (Yaw)

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 6 || msg->buttons.size() < 6) return;
        
        // --- 速度として入力値を保持 ---
        vel_x_ = msg->axes[1]; 
        vel_y_ = msg->axes[0]; 
        
        vel_z_   = msg->axes[4]; 
        vel_phi_ = msg->axes[3]; 

        vel_theta_ = 0.0f;
        if (msg->buttons[4]) vel_theta_ += 1.0f; // L1
        if (msg->buttons[5]) vel_theta_ -= 1.0f; // R1

        float l2 = (1.0f - msg->axes[2]) / 2.0f; 
        float r2 = (1.0f - msg->axes[5]) / 2.0f; 
        vel_psi_ = l2 - r2; 
    }

    void ik_timer_callback() {
        const float pos_gain = 5.0f;  
        const float rot_gain = 0.05f; 

        target_pose_[0] += vel_x_ * pos_gain;
        target_pose_[1] += vel_y_ * pos_gain;
        target_pose_[2] += vel_z_ * pos_gain;
        target_pose_[3] += vel_phi_ * rot_gain;
        // target_pose_[4] += vel_theta_ * rot_gain;
        // target_pose_[5] += vel_psi_ * rot_gain;
        RCLCPP_INFO(this->get_logger(), "Current Pose -> X: %.2f, Y: %.2f, Z: %.2f", target_pose_[0], target_pose_[1], target_pose_[2]);
        if (!ik_client_->service_is_ready()) {
            return;
        }

        auto request = std::make_shared<catchrobo2026_msgs::srv::InverseKinematics::Request>();
        for(int i = 0; i < 6; ++i) {
            request->target_pose[i] = target_pose_[i];
        }
        

        ik_client_->async_send_request(request,
            std::bind(&JoyControllerNode::ik_response_callback, this, std::placeholders::_1));
    }

    void ik_response_callback(rclcpp::Client<catchrobo2026_msgs::srv::InverseKinematics>::SharedFuture future) {
        auto response = future.get();
        
        // 【変更】ここではパブリッシュせず、最新の角度を変数に保存するだけにする
        for(int i = 0; i < 4; ++i) {
            latest_joint_angles_[i] = response->joint_angles[i];
        }
        has_valid_joints_ = true; // 初回の計算が完了したフラグを立てる
    }

    // 【追加】一定周期で実行されるパブリッシュ専用のコールバック
    void publish_timer_callback() {
        // まだ一度も逆運動学の計算結果を受け取っていない場合はスキップ
        if (!has_valid_joints_) {
            return;
        }

        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(4);
        for(int i = 0; i < 4; ++i) {
            msg.data[i] = latest_joint_angles_[i];
        }
        joint_pub_->publish(msg);
    }

    // --- 変数定義 ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
    rclcpp::Client<catchrobo2026_msgs::srv::InverseKinematics>::SharedPtr ik_client_;
    
    rclcpp::TimerBase::SharedPtr ik_timer_;      // IK計算要求用タイマー
    rclcpp::TimerBase::SharedPtr publish_timer_; // パブリッシュ用タイマー

    float target_pose_[6];
    
    // 【追加】最新の関節角度を保持する変数
    float latest_joint_angles_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool has_valid_joints_ = false; 
    
    float vel_x_ = 0.0f;
    float vel_y_ = 0.0f;
    float vel_z_ = 0.0f;
    float vel_phi_ = 0.0f;
    float vel_theta_ = 0.0f;
    float vel_psi_ = 0.0f;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}