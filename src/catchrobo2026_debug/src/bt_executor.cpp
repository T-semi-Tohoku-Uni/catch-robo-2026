#include <chrono>
#include <memory>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto server = std::make_shared<BT::TreeExecutionServer>(rclcpp::NodeOptions{});

  // Use a finite spin timeout to avoid the executor deadlock described by the
  // BehaviorTree.ROS2 sample when publishers are removed dynamically.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  executor.add_node(server->node());
  executor.spin();
  executor.remove_node(server->node());

  rclcpp::shutdown();
  return 0;
}
