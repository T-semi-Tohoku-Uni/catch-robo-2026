cmake_minimum_required(VERSION 3.8)
project(nav_director)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 3D経路生成ノードの追加依存関係
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

# 実行ファイルの追加
add_executable(path_generator_3d src/path_generator_3d.cpp)

# インクルードディレクトリの設定 (Eigen3用)
target_include_directories(path_generator_3d PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# 依存パッケージのリンク
ament_target_dependencies(path_generator_3d
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  catchrobo2026_msgs
  visualization_msgs
)

# ros2 run 等で実行できるようにインストール
install(TARGETS path_generator_3d
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