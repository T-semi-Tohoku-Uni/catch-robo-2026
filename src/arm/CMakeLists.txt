cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/arm_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(kinematics test/test_kinematics.py)
endif()

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Mesh assets may be supplied locally and are ignored by Git.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
