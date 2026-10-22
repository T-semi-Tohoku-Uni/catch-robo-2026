cmake_minimum_required(VERSION 3.8)
project(path_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Also permits algorithm-only builds on machines without ROS.
option(PATH_PLANNING_WITH_ROS "Build ROS 2 adapter" ON)
option(PATH_PLANNING_WITH_COLLISION "Build URDF/FCL collision adapter" ${PATH_PLANNING_WITH_ROS})
add_library(informed_rrt_star src/informed_rrt_star.cpp)
target_include_directories(informed_rrt_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(PATH_PLANNING_WITH_COLLISION)
  find_package(fcl REQUIRED)
  find_package(urdfdom REQUIRED)
  add_library(arm_collision_checker src/arm_collision_checker.cpp)
  target_include_directories(arm_collision_checker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(arm_collision_checker PRIVATE fcl urdfdom::urdfdom_model)
  install(TARGETS arm_collision_checker EXPORT informed_rrt_starTargets
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_informed_rrt_star test/test_informed_rrt_star.cpp)
  target_link_libraries(test_informed_rrt_star informed_rrt_star)
  add_test(NAME informed_rrt_star COMMAND test_informed_rrt_star)
endif()

if(PATH_PLANNING_WITH_ROS)
  if(NOT PATH_PLANNING_WITH_COLLISION)
    message(FATAL_ERROR "ROS adapter requires PATH_PLANNING_WITH_COLLISION=ON")
  endif()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  add_executable(path_planner_node src/main.cpp src/planner_node.cpp)
  target_link_libraries(path_planner_node informed_rrt_star arm_collision_checker)
  ament_target_dependencies(path_planner_node rclcpp sensor_msgs trajectory_msgs std_srvs)
  add_executable(path_preview_node src/preview_main.cpp src/path_preview_node.cpp)
  target_link_libraries(path_preview_node arm_collision_checker)
  ament_target_dependencies(path_preview_node rclcpp trajectory_msgs visualization_msgs)
  install(TARGETS path_preview_node DESTINATION lib/${PROJECT_NAME})
  if(BUILD_TESTING)
    find_package(robot_arm_description REQUIRED)
    find_program(XACRO_EXECUTABLE xacro REQUIRED)
    set(ARM_XACRO "${robot_arm_description_DIR}/../urdf/robot_arm.xacro")
    set(ARM_TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/test_robot_arm.urdf")
    add_custom_command(OUTPUT "${ARM_TEST_URDF}"
      COMMAND "${XACRO_EXECUTABLE}" "${ARM_XACRO}" -o "${ARM_TEST_URDF}"
      DEPENDS "${ARM_XACRO}" VERBATIM)
    add_custom_target(test_robot_arm_urdf DEPENDS "${ARM_TEST_URDF}")
    add_executable(test_arm_collision_checker test/test_arm_collision_checker.cpp)
    target_link_libraries(test_arm_collision_checker arm_collision_checker informed_rrt_star)
    add_dependencies(test_arm_collision_checker test_robot_arm_urdf)
    add_test(NAME arm_collision_checker COMMAND test_arm_collision_checker "${ARM_TEST_URDF}")
  endif()
  install(TARGETS path_planner_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS informed_rrt_star EXPORT informed_rrt_starTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
if(PATH_PLANNING_WITH_ROS)
  ament_export_dependencies(fcl urdfdom)
  ament_export_targets(informed_rrt_starTargets HAS_LIBRARY_TARGET)
  ament_package()
endif()
