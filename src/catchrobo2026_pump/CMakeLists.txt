cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_pump)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)

add_executable(pump_controller_node src/pump_controller_node.cpp)
target_compile_features(pump_controller_node PRIVATE cxx_std_17)
target_include_directories(pump_controller_node PRIVATE include)
ament_target_dependencies(pump_controller_node
  rclcpp
  rcl_interfaces
  std_msgs
  catchrobo2026_msgs
)

install(TARGETS
  pump_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_check_suction test/test_check_suction.py
    TIMEOUT 90
    ENV PUMP_NODE_EXECUTABLE=$<TARGET_FILE:pump_controller_node>)
endif()

ament_package()
