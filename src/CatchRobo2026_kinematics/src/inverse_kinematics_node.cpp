#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <memory>
#include "ros2_inverse_kinematics/robot_kinematics.h"


#include "catchrobo2026_msgs/srv/inverse_kinematics.hpp"

using namespace std;


using IKSrv = catchrobo2026_msgs::srv::InverseKinematics;


void calc_inverse_kinematics(const std::shared_ptr<IKSrv::Request> request, 
                             std::shared_ptr<IKSrv::Response> response)
{

  robot_kinematics robot_kin;


  float target_pos[6] = {
      request->target_pose[0], request->target_pose[1], request->target_pose[2],
      request->target_pose[3], request->target_pose[4], request->target_pose[5]
  };
  
  float joint_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  //逆運動学の計算を実行
  robot_kin.inverse_kinematics(target_pos, joint_angle);

  //計算結果をレスポンスに詰める
  response->joint_angles[0] = joint_angle[0];
  response->joint_angles[1] = joint_angle[1];
  response->joint_angles[2] = joint_angle[2];
  response->joint_angles[3] = joint_angle[3];

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calculated Inverse Kinematics successfully");
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response:");
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inverse_kinematics_server");

  rclcpp::Service<IKSrv>::SharedPtr service = node->create_service<IKSrv>("inverse_kinematics", &calc_inverse_kinematics);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inverse kinematics ready");

  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}