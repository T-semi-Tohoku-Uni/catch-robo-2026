cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(follow_route_bt_plugin SHARED
  src/follow_route_action.cpp
)
target_include_directories(follow_route_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(follow_route_bt_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(follow_route_bt_plugin
  behaviortree_ros2
  catchrobo2026_msgs
  rclcpp
  rclcpp_action
)

add_library(generate_route_bt_plugin SHARED
  src/generate_route_service.cpp
)
target_include_directories(generate_route_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(generate_route_bt_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(generate_route_bt_plugin
  behaviortree_ros2
  catchrobo2026_msgs
  rclcpp
)

add_executable(bt_executor
  src/bt_executor.cpp
)
ament_target_dependencies(bt_executor
  behaviortree_ros2
  btcpp_ros2_interfaces
  rclcpp
  rclcpp_action
)

install(TARGETS
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  follow_route_bt_plugin
  generate_route_bt_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
