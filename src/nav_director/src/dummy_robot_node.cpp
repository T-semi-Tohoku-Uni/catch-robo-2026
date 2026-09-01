#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>

// 運動学ライブラリのヘッダーをインクルード
#include "ros2_inverse_kinematics/robot_kinematics.h"

class DummyRobotNode : public rclcpp::Node {
public:
    DummyRobotNode() : Node("dummy_robot_node") {
        // パブリッシャーとサブスクライバーの初期化
        pub_current_joints_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("current_joints", 10);
        pub_current_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
        
        sub_target_joints_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_joint_angles", 10,
            std::bind(&DummyRobotNode::targetJointCallback, this, std::placeholders::_1));

        // --- 初期位置を [X, Y, Z, Yaw] で受け取る ---
        // ※単位はご自身の運動学ライブラリに合わせてください（例: X,Y,Zはミリメートルかメートルか）
        std::vector<double> default_pose = {600.0, 200.0, 200.0, 0.0}; // X, Y, Z, Yaw
        this->declare_parameter<std::vector<double>>("initial_pose", default_pose);
        
        std::vector<double> initial_pose_param;
        this->get_parameter("initial_pose", initial_pose_param);

        // 逆運動学用の配列を作成
        float target_posrot[6] = {0.0};
        target_posrot[0] = static_cast<float>(initial_pose_param[0]); // X
        target_posrot[1] = static_cast<float>(initial_pose_param[1]); // Y
        target_posrot[2] = static_cast<float>(initial_pose_param[2]); // Z
        target_posrot[3] = static_cast<float>(initial_pose_param[3]); // Yaw (PHI)
        
        // path_follower_node の実装に合わせて固定値を設定
        target_posrot[4] = -M_PI / 2.0F; // THE
        target_posrot[5] = 0.0F;         // PSI

        // 逆運動学を解いて初期ジョイント角度を計算
        float initial_joints[4] = {0.0};
        kin_.inverse_kinematics(target_posrot, initial_joints);

        // 初期ジョイント角度として保存
        current_joints_.resize(4);
        for (int i = 0; i < 4; ++i) {
            current_joints_[i] = initial_joints[i];
        }

        RCLCPP_INFO(this->get_logger(), "Dummy Robot Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "Input Initial Pose : [X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f]", 
                    target_posrot[0], target_posrot[1], target_posrot[2], target_posrot[3]);
        RCLCPP_INFO(this->get_logger(), "Calculated Joints  : [%.2f, %.2f, %.2f, %.2f]", 
                    current_joints_[0], current_joints_[1], current_joints_[2], current_joints_[3]);

        // 50Hz (20ms) でパブリッシュするタイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DummyRobotNode::timerCallback, this));
    }

private:
    void targetJointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
            // 受信した目標角度をそのまま現在角度として更新
            for (int i = 0; i < 4; ++i) {
                current_joints_[i] = msg->data[i];
            }
        }
    }

    void timerCallback() {
        // 1. 現在のジョイント角度をパブリッシュ
        std_msgs::msg::Float32MultiArray msg_out;
        msg_out.data = current_joints_;
        pub_current_joints_->publish(msg_out);

        // 2. 順運動学で現在位置を再計算 (RViz表示用)
        float joint_angles[4];
        for (int i = 0; i < 4; ++i) {
            joint_angles[i] = current_joints_[i];
        }
        
        float current_posrot[6] = {0.0};
        kin_.forward_kinematics(current_posrot, joint_angles);

        // 3. PoseStampedメッセージを作成してパブリッシュ
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        // RViz表示用にメートル変換が必要な場合は / 1000.0 をつけてください
        pose_msg.pose.position.x = current_posrot[0] / 1000.0;
        pose_msg.pose.position.y = current_posrot[1] / 1000.0;
        pose_msg.pose.position.z = current_posrot[2] / 1000.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_posrot[3]);
        pose_msg.pose.orientation = tf2::toMsg(q);

        pub_current_pose_->publish(pose_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_current_joints_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_current_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_target_joints_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<float> current_joints_;
    robot_kinematics kin_; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyRobotNode>());
    rclcpp::shutdown();
    return 0;
}