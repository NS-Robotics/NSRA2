cmake_minimum_required(VERSION 3.5)
project(robot_descriptions)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "common/msg/Pos.msg"
  "common/msg/Init.msg"
  "common/msg/Stop.msg"
  "common/msg/Vel.msg"
)

install(
  DIRECTORY nsra2/config nsra2/gazebo nsra2/launch nsra2/meshes nsra2/ros2_control nsra2/urdf
  DESTINATION share/${PROJECT_NAME}/nsra2
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/nsra2/env-hooks/nsra2_description.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()