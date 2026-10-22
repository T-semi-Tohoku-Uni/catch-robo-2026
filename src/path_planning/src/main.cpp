#include <exception>
#include <memory>
#include "path_planning/planner_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<path_planning::PlannerNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("path_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
