#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <memory>
#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "catchrobo2026_msgs/srv/inverse_kinematics.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std;
using IKSrv = catchrobo2026_msgs::srv::InverseKinematics;

class InverseKinematicsNode : public rclcpp::Node {
public:
  InverseKinematicsNode() : Node("inverse_kinematics_server") {
    // マーカー用のパブリッシャーを作成
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ik_markers", 10);
    
    // サービスの作成
    service_ = this->create_service<IKSrv>(
      "inverse_kinematics",
      std::bind(&InverseKinematicsNode::calc_inverse_kinematics, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Inverse kinematics ready");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<IKSrv>::SharedPtr service_;
  robot_kinematics robot_kin;

  void calc_inverse_kinematics(const std::shared_ptr<IKSrv::Request> request,
                               std::shared_ptr<IKSrv::Response> response) {
    float target_pos[6] = {
        request->target_pose[0], request->target_pose[1], request->target_pose[2],
        request->target_pose[3], request->target_pose[4], request->target_pose[5]
    };
    float joint_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 逆運動学の計算
    robot_kin.inverse_kinematics(target_pos, joint_angle);

    response->joint_angles[0] = joint_angle[0];
    response->joint_angles[1] = joint_angle[1];
    response->joint_angles[2] = joint_angle[2];
    response->joint_angles[3] = joint_angle[3];

    // 計算後にマーカーをパブリッシュ（target_pos も渡すように修正）
    publish_markers(joint_angle, target_pos);
  }

  // target_pos を引数に追加
  void publish_markers(float *joint_angle, float *target_pos) {
    float positions[5][3];
    robot_kin.get_joint_positions(joint_angle, positions);

    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time now = this->now();

    // 1. 各関節のリンクマーカー（矢印）の生成
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now;
      marker.ns = "ik_links";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = (positions[i][0] + robot_pos[0]) / 1000.0;
      p_start.y = (positions[i][1] + robot_pos[1]) / 1000.0;
      p_start.z = (positions[i][2] + robot_pos[2]) / 1000.0;
      p_end.x = (positions[i+1][0] + robot_pos[0]) / 1000.0;
      p_end.y = (positions[i+1][1] + robot_pos[1]) / 1000.0;
      p_end.z = (positions[i+1][2] + robot_pos[2]) / 1000.0;

      marker.points.push_back(p_start);
      marker.points.push_back(p_end);

      marker.scale.x = 0.02; // シャフトの直径
      marker.scale.y = 0.04; // 矢印の頭の直径
      marker.scale.z = 0.04; // 矢印の頭の長さ

      marker.color.a = 1.0;
      marker.color.r = (i == 0) ? 1.0 : 0.0;
      marker.color.g = (i == 1) ? 1.0 : 0.0;
      marker.color.b = (i >= 2) ? 1.0 : 0.0;

      marker_array.markers.push_back(marker);
    }

    // 2. 目標地点のマーカー（球体）の生成
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = "map";
    target_marker.header.stamp = now;
    target_marker.ns = "ik_target"; // リンクと名前空間を分ける
    target_marker.id = 4;           // IDが被らないようにする
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;

    // target_posはフィールド座標系[mm]なので、そのまま[m]にして表示
    target_marker.pose.position.x = target_pos[0] / 1000.0;
    target_marker.pose.position.y = target_pos[1] / 1000.0;
    target_marker.pose.position.z = target_pos[2] / 1000.0;

    // 球体の大きさ（直径5cm）
    target_marker.scale.x = 0.05;
    target_marker.scale.y = 0.05;
    target_marker.scale.z = 0.05;

    // 色を設定（例として目立つようにマゼンタ色）
    target_marker.color.a = 0.8; // 少し半透明
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 1.0;

    marker_array.markers.push_back(target_marker);

    // まとめてパブリッシュ
    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}