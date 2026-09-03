#pragma once

#include <string>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "catchrobo2026_msgs/srv/generate_route.hpp"

namespace catchrobo2026_debug
{

class GenerateRouteService
  : public BT::RosServiceNode<catchrobo2026_msgs::srv::GenerateRoute>
{
public:
  using Service = catchrobo2026_msgs::srv::GenerateRoute;
  using Base = BT::RosServiceNode<Service>;

  GenerateRouteService(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace catchrobo2026_debug
