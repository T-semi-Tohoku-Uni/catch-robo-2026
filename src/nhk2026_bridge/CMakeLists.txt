cmake_minimum_required(VERSION 3.8)
project(nhk2026_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(backward_ros REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

# can library
add_library(can_library
  src/ros2can_bridge.cpp
)

# # joy2vel
# add_executable(joy_vel_converter
#   src/joy2vel.cpp
# )

# ament_target_dependencies(joy_vel_converter
#   rclcpp
#   rclcpp_lifecycle
#   sensor_msgs
#   geometry_msgs
# )

# install(
#   TARGETS joy_vel_converter
#   DESTINATION lib/${PROJECT_NAME}
# )

# test bridge
add_executable(test_canbridge
  src/test_canbridge.cpp
)

ament_target_dependencies(test_canbridge
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

target_link_libraries(test_canbridge
  can_library
)

install(
  TARGETS test_canbridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(nhk2026_canbridge
  src/nhk2026_canbridge.cpp
)

ament_target_dependencies(nhk2026_canbridge
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  lifecycle_msgs
)

target_link_libraries(nhk2026_canbridge
  can_library
)

install(
  TARGETS nhk2026_canbridge
  DESTINATION lib/${PROJECT_NAME}
)

# add install
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/test_vcan_bridge.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
