cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(PATH_PLANNER_WITH_ROS "Build the ROS 2 preview adapter" ON)
find_package(Threads REQUIRED)
if(PATH_PLANNER_WITH_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
endif()

set(PATH_PLANNER_CORE_SOURCES
  src/yaml_lite.cpp
  src/robot_model.cpp
  src/collision_checker.cpp
  src/rrt_connect.cpp
  src/path_post_processor.cpp
  src/dynamics_model.cpp
  src/torque_aware_cost.cpp
  src/toppra_parameterizer.cpp
  src/jerk_limiter.cpp
  src/trajectory_validator.cpp
  src/trajectory_pipeline.cpp
  src/trajectory_node.cpp
)

add_library(rrt_connect ${PATH_PLANNER_CORE_SOURCES})
target_include_directories(rrt_connect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(rrt_connect PUBLIC cxx_std_17)
target_link_libraries(rrt_connect PUBLIC Threads::Threads)

add_executable(trajectory_node src/trajectory_main.cpp)
target_link_libraries(trajectory_node rrt_connect)

if(PATH_PLANNER_WITH_ROS)
  target_compile_definitions(trajectory_node PRIVATE PATH_PLANNER_HAS_ROS2=1)
  ament_target_dependencies(trajectory_node
    rclcpp
    sensor_msgs
    std_msgs
    trajectory_msgs
    diagnostic_msgs
    visualization_msgs
    nav_msgs
    geometry_msgs
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

install(TARGETS rrt_connect trajectory_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)

set(PATH_PLANNER_TESTS
  test_rrt_connect
  test_path_post_processor
  test_dynamics_model
  test_toppra_parameterizer
  test_trajectory_validator
  test_trajectory_pipeline
)

foreach(test_name IN LISTS PATH_PLANNER_TESTS)
  add_executable(${test_name} test/${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE rrt_connect)
  add_test(
    NAME ${test_name}
    COMMAND ${test_name}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endforeach()

if(PATH_PLANNER_WITH_ROS)
  find_package(inverse_kinematics_package REQUIRED)
  add_executable(test_production_kinematics test/test_production_kinematics.cpp)
  target_link_libraries(test_production_kinematics rrt_connect)
  ament_target_dependencies(test_production_kinematics inverse_kinematics_package)
  add_test(NAME test_production_kinematics COMMAND test_production_kinematics)
endif()
endif()

if(PATH_PLANNER_WITH_ROS)
  ament_export_include_directories(include)
  ament_export_libraries(rrt_connect)
  ament_package()
endif()
