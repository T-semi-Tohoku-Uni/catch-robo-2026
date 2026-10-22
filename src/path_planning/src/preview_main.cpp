#include <exception>
#include <memory>
#include "path_planning/path_preview_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<path_planning::PathPreviewNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("path_preview"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
