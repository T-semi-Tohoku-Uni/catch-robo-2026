#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <mutex>
#include <vector>

// 提供された運動学ライブラリと独自メッセージ
#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "catchrobo2026_msgs/action/follow_route.hpp"

using FollowRoute = catchrobo2026_msgs::action::FollowRoute;
using GoalHandleFollowRoute = rclcpp_action::ServerGoalHandle<FollowRoute>;

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower_node"), current_path_index_(0) {
        // パブリッシャーとサブスクライバーの初期化
        pub_joints_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);
        
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "route", 10, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
            
        sub_current_joints_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "current_joints", 10, std::bind(&PathFollowerNode::jointCallback, this, std::placeholders::_1));

        // アクションサーバーの初期化 (20Hzでの制御ループ用)
        action_server_ = rclcpp_action::create_server<FollowRoute>(
            this,
            "follow_route",
            std::bind(&PathFollowerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PathFollowerNode::handleCancel, this, std::placeholders::_1),
            std::bind(&PathFollowerNode::handleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Path Follower Node Initialized.");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_path_ = *msg;
        current_path_index_ = 0; // 新しい経路を受信したらインデックスをリセット
    }

    void jointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (msg->data.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                current_joint_angles_[i] = msg->data[i];
            }
        }
    }

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowRoute::Goal> goal) {
        (void)uuid;
        if (!goal->start) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFollowRoute> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFollowRoute> goal_handle) {
        std::thread{std::bind(&PathFollowerNode::executeLoop, this, std::placeholders::_1), goal_handle}.detach();
    }

    void executeLoop(const std::shared_ptr<GoalHandleFollowRoute> goal_handle) {
        rclcpp::Rate loop_rate(20); // 20Hzで実行
        auto feedback = std::make_shared<FollowRoute::Feedback>();
        auto result = std::make_shared<FollowRoute::Result>();

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            float current_posrot[6] = {0.0};
            float target_posrot[6] = {0.0};
            float target_joints[4] = {0.0};
            
            nav_msgs::msg::Path local_path;
            float local_joints[4];

            // データの排他制御コピー
            {
                std::lock_guard<std::mutex> lock_p(path_mutex_);
                std::lock_guard<std::mutex> lock_j(joint_mutex_);
                local_path = current_path_;
                for(int i=0; i<4; i++) local_joints[i] = current_joint_angles_[i];
            }

            if (local_path.poses.empty()) {
                loop_rate.sleep();
                continue;
            }

            // 1. 順運動学で現在地を計算
            kin_.forward_kinematics(current_posrot, local_joints);
            
            double cx = current_posrot[0]; // X
            double cy = current_posrot[1]; // Y
            double cz = current_posrot[2]; // Z

            // 2. 0.1m先の目標経路探索
            bool found_target = false;
            for (size_t i = current_path_index_; i < local_path.poses.size(); ++i) {
                const auto& pose = local_path.poses[i].pose;
                double dx = pose.position.x - cx;
                double dy = pose.position.y - cy;
                double dz = pose.position.z - cz;
                
                // 3次元空間でのユークリッド距離 $d = \sqrt{dx^2 + dy^2 + dz^2}$ を計算
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (distance >= 0.1) {
                    target_posrot[0] = pose.position.x;
                    target_posrot[1] = pose.position.y;
                    target_posrot[2] = pose.position.z;

                    // クォータニオンからPHI（ヨー角）を取得
                    tf2::Quaternion q;
                    tf2::fromMsg(pose.orientation, q);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    
                    target_posrot[3] = static_cast<float>(yaw); // PHI
                    target_posrot[4] = -M_PI / 2.0F;            // THE (robot_kinematicsの実装に依存)
                    target_posrot[5] = 0.0F;                    // PSI

                    current_path_index_ = i;
                    found_target = true;
                    break;
                }
            }

            // 目標が見つからなかった場合（終点到達など）は最終ウェイポイントを維持
            if (!found_target) {
                const auto& last_pose = local_path.poses.back().pose;
                target_posrot[0] = last_pose.position.x;
                target_posrot[1] = last_pose.position.y;
                target_posrot[2] = last_pose.position.z;
                
                tf2::Quaternion q;
                tf2::fromMsg(last_pose.orientation, q);
                double r, p, y;
                tf2::Matrix3x3(q).getRPY(r, p, y);
                target_posrot[3] = static_cast<float>(y);
                target_posrot[4] = -M_PI / 2.0F;
                target_posrot[5] = 0.0F;
                
                // 完了判定
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Reached the end of the path.");
                return;
            }

            // 3. 逆運動学で目標ジョイント角を計算
            kin_.inverse_kinematics(target_posrot, target_joints);

            // 4. Float32MultiArrayで出力
            std_msgs::msg::Float32MultiArray msg_out;
            msg_out.data.resize(4);
            for (int i = 0; i < 4; ++i) {
                msg_out.data[i] = target_joints[i];
            }
            pub_joints_->publish(msg_out);

            // フィードバックの送信 (省略可)
            feedback->distance_remaining = local_path.poses.size() - current_path_index_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
    }

    robot_kinematics kin_; // 運動学クラスのインスタンス
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_joints_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_current_joints_;
    rclcpp_action::Server<FollowRoute>::SharedPtr action_server_;

    std::mutex path_mutex_;
    std::mutex joint_mutex_;
    nav_msgs::msg::Path current_path_;
    float current_joint_angles_[4] = {0.0};
    size_t current_path_index_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}