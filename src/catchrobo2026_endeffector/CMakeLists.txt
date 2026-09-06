cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_endeffector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)

add_executable(endeffector_state_node src/endeffector_state_node.cpp)
ament_target_dependencies(endeffector_state_node
  rclcpp
  std_msgs
  catchrobo2026_msgs
)

install(TARGETS
  endeffector_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
