cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_hand_operated)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)

# ==========================================
# 追加した依存パッケージ
# ==========================================
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(inverse_kinematics_package REQUIRED)

# ==========================================
# 実行ファイルの追加と依存パッケージのリンク
# ==========================================

# 1. joy_controller_node
add_executable(joy_controller_node src/joy_controller_node.cpp)
ament_target_dependencies(joy_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  catchrobo2026_msgs
  geometry_msgs            # 追加
  tf2                      # 追加
  tf2_geometry_msgs        # 追加
  inverse_kinematics_package
)

# 2. pump_state_node 
add_executable(pump_state_node src/pump_state_node.cpp)
ament_target_dependencies(pump_state_node
  rclcpp
  std_msgs
  catchrobo2026_msgs
)

# ==========================================
# インストール設定
# ==========================================
install(TARGETS
  joy_controller_node
  pump_state_node   
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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