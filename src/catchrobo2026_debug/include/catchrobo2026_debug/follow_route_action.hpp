#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "catchrobo2026_msgs/action/follow_route.hpp"

namespace catchrobo2026_debug
{

class FollowRouteAction
  : public BT::RosActionNode<catchrobo2026_msgs::action::FollowRoute>
{
public:
  using Action = catchrobo2026_msgs::action::FollowRoute;
  using Base = BT::RosActionNode<Action>;

  FollowRouteAction(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onResultReceived(const WrappedResult & result) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;
};

}  // namespace catchrobo2026_debug
