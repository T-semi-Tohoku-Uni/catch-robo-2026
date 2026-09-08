cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_sequence)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(catchrobo2026_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(sequence_config src/sequence_config.cpp)
target_compile_features(sequence_config PUBLIC cxx_std_17)
target_include_directories(sequence_config PUBLIC include)
target_link_libraries(sequence_config yaml-cpp)

add_executable(sequence_node src/sequence_node.cpp)
target_link_libraries(sequence_node sequence_config)
ament_target_dependencies(sequence_node
  ament_index_cpp rclcpp rclcpp_action rcl_interfaces std_srvs catchrobo2026_msgs)

install(TARGETS sequence_config sequence_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_route_handoff test/test_route_handoff.py TIMEOUT 120)
  ament_add_gtest(test_sequence_config test/test_sequence_config.cpp)
  target_link_libraries(test_sequence_config sequence_config)
  ament_add_gtest(test_builtin_sequences test/test_builtin_sequences.cpp)
  target_link_libraries(test_builtin_sequences sequence_config)
  target_compile_definitions(test_builtin_sequences PRIVATE
    SEQUENCE_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/sequences.yaml")
endif()

ament_package()
