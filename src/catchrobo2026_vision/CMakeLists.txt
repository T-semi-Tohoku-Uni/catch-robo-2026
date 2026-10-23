cmake_minimum_required(VERSION 3.8)
project(catchrobo2026_vision)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/vision_http_bridge.py scripts/make_test_video.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(vision_http
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_http_bridge.py
    TIMEOUT 60 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
