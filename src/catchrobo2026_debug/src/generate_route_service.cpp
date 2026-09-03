#include "catchrobo2026_debug/generate_route_service.hpp"

#include "behaviortree_ros2/plugins.hpp"

namespace catchrobo2026_debug
{

GenerateRouteService::GenerateRouteService(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: Base(name, config, params)
{
  setServiceName("/generate_route");
}

BT::PortsList GenerateRouteService::providedPorts()
{
  return {
    BT::InputPort<double>("x", "Goal x position [mm]"),
    BT::InputPort<double>("y", "Goal y position [mm]"),
    BT::InputPort<double>("z", "Goal z position [mm]"),
    BT::InputPort<double>("phi", "Goal yaw angle [rad]"),
    BT::OutputPort<bool>("success", "Whether route generation succeeded"),
  };
}

bool GenerateRouteService::setRequest(Request::SharedPtr & request)
{
  const auto x = getInput<double>("x");
  const auto y = getInput<double>("y");
  const auto z = getInput<double>("z");
  const auto phi = getInput<double>("phi");

  if (!x || !y || !z || !phi) {
    RCLCPP_ERROR(
      logger(), "%s: input ports [x], [y], [z], and [phi] are required",
      name().c_str());
    return false;
  }

  request->x = x.value();
  request->y = y.value();
  request->z = z.value();
  request->phi = phi.value();
  return true;
}

BT::NodeStatus GenerateRouteService::onResponseReceived(
  const Response::SharedPtr & response)
{
  const bool succeeded = response && response->success;
  setOutput("success", succeeded);

  if (succeeded) {
    RCLCPP_INFO(logger(), "%s: route generation succeeded", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "%s: route generation service returned failure", name().c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GenerateRouteService::onFailure(BT::ServiceNodeErrorCode error)
{
  setOutput("success", false);
  RCLCPP_ERROR(
    logger(), "%s: GenerateRoute service failed: %s", name().c_str(), BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace catchrobo2026_debug

CreateRosNodePlugin(catchrobo2026_debug::GenerateRouteService, "GenerateRoute");
