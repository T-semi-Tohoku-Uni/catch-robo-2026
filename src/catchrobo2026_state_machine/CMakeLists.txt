cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(path_planner REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)
find_package(canrpc_bridge REQUIRED)

add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
  rclcpp
  rcl_interfaces
  std_msgs
  std_srvs
  catchrobo2026_msgs
  canrpc_bridge
)

add_executable(arm_target_client src/arm_target_client.cpp)
target_compile_features(arm_target_client PRIVATE cxx_std_17)
ament_target_dependencies(arm_target_client
  rclcpp rclcpp_action rcl_interfaces std_msgs std_srvs canrpc_bridge catchrobo2026_msgs path_planner)

install(TARGETS
  arm_target_client
  state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_arm_target_client test/test_arm_target_client.py TIMEOUT 120)
  ament_add_pytest_test(test_trajectory_owner test/test_trajectory_owner.py TIMEOUT 120)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
