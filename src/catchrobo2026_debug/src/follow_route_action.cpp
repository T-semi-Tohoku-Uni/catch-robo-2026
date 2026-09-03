#include "catchrobo2026_debug/follow_route_action.hpp"

#include "behaviortree_ros2/plugins.hpp"

namespace catchrobo2026_debug
{

FollowRouteAction::FollowRouteAction(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: Base(name, config, params)
{
  setActionName("/follow_route");
}

BT::PortsList FollowRouteAction::providedPorts()
{
  return {
    BT::InputPort<bool>("start", true, "Whether to start following the route"),
    BT::OutputPort<float>("distance_remaining", "Remaining route points"),
  };
}

bool FollowRouteAction::setGoal(Goal & goal)
{
  const auto start = getInput<bool>("start");
  if (!start) {
    RCLCPP_ERROR(logger(), "%s: invalid input port [start]", name().c_str());
    return false;
  }

  goal.start = start.value();
  if (!goal.start) {
    RCLCPP_ERROR(logger(), "%s: [start] must be true", name().c_str());
    return false;
  }

  return true;
}

BT::NodeStatus FollowRouteAction::onFeedback(
  const std::shared_ptr<const Feedback> feedback)
{
  if (feedback) {
    setOutput("distance_remaining", feedback->distance_remaining);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowRouteAction::onResultReceived(const WrappedResult & result)
{
  if (
    result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    result.result && result.result->success)
  {
    RCLCPP_INFO(logger(), "%s: route following succeeded", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    logger(), "%s: route following returned an unsuccessful result", name().c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowRouteAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    logger(), "%s: FollowRoute action failed: %s", name().c_str(), BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

void FollowRouteAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted", name().c_str());
}

}  // namespace catchrobo2026_debug

CreateRosNodePlugin(catchrobo2026_debug::FollowRouteAction, "FollowRoute");
