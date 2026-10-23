cmake_minimum_required(VERSION 3.8)
project(red_cylinder_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(red_cylinder_detector_node src/red_cylinder_detector_node.cpp)
target_compile_features(red_cylinder_detector_node PUBLIC cxx_std_17)
target_compile_options(red_cylinder_detector_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(red_cylinder_detector_node
  rclcpp rcl_interfaces sensor_msgs std_msgs geometry_msgs)
target_include_directories(red_cylinder_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(red_cylinder_detector_node ${OpenCV_LIBS})
install(TARGETS red_cylinder_detector_node DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(detector_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_detector.py
      $<TARGET_FILE:red_cylinder_detector_node>
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
