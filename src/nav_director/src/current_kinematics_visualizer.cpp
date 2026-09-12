#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

// 運動学ライブラリのインクルード
#include "ros2_inverse_kinematics/robot_kinematics.h"

class CurrentKinematicsVisualizer : public rclcpp::Node {
public:
    CurrentKinematicsVisualizer() : Node("current_kinematics_visualizer") {
        // MarkerArray用のパブリッシャー
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("current_robot_markers", 10);
        
        // 順運動学の結果(x, y, z, yaw)をパブリッシュするパブリッシャーを追加
        pose_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("current_pose3d", 10);
        
        // current_jointsのサブスクライバー
        joint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "current_joints", 10,
            std::bind(&CurrentKinematicsVisualizer::jointCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Current Kinematics Visualizer initialized.");
    }

private:
    void jointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Expected at least 4 joint angles.");
            return;
        }

        float joint_angle[4] = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
        
        // 追加: 順運動学の計算とパブリッシュ
        float posrot[6] = {0.0f};
        robot_kin_.forward_kinematics(posrot, joint_angle);

        std_msgs::msg::Float32MultiArray pose_msg;
        // posrotのインデックスは 0:X, 1:Y, 2:Z, 3:PHI(Yaw)
        pose_msg.data = {posrot[0], posrot[1], posrot[2], posrot[3]};
        pose_pub_->publish(pose_msg);

        // 既存のマーカー描画用処理
        float positions[6][3];
        
        // 運動学ライブラリのHomogeneous transformチェーンを利用して全リンクの座標を一括取得
        robot_kin_.get_joint_positions(joint_angle, positions);

        publishMarkers(positions);
    }

    void publishMarkers(float positions[6][3]) {
        visualization_msgs::msg::MarkerArray marker_array;
        rclcpp::Time now = this->now();

        // 基準座標用マーカー（IKサーバーの実装に準拠し、不要なマーカーを削除）
        visualization_msgs::msg::Marker obsolete_base_marker;
        obsolete_base_marker.header.frame_id = "map";
        obsolete_base_marker.header.stamp = now;
        obsolete_base_marker.ns = "current_links";
        obsolete_base_marker.id = 0;
        obsolete_base_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(obsolete_base_marker);

        // 各リンクを矢印として描画
        for (int i = 1; i < 5; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now;
            marker.ns = "current_links";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;

            // mmからmへの単位変換
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = positions[i][0] / 1000.0;
            p_start.y = positions[i][1] / 1000.0;
            p_start.z = positions[i][2] / 1000.0;

            p_end.x = positions[i+1][0] / 1000.0;
            p_end.y = positions[i+1][1] / 1000.0;
            p_end.z = positions[i+1][2] / 1000.0;

            const double length = std::sqrt(
                std::pow(p_end.x - p_start.x, 2) +
                std::pow(p_end.y - p_start.y, 2) +
                std::pow(p_end.z - p_start.z, 2));
            if (length < 1e-9) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_array.markers.push_back(marker);
                continue;
            }
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);

            marker.scale.x = 0.02; // 軸の太さ
            marker.scale.y = 0.04; // 矢印の頭の太さ
            marker.scale.z = 0.04; // 矢印の頭の長さ
            marker.color.a = 1.0;
            
            // 色の割り当て
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 0.8; 

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_sub_;
    robot_kinematics robot_kin_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentKinematicsVisualizer>());
    rclcpp::shutdown();
    return 0;
}
