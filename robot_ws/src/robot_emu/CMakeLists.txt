cmake_minimum_required(VERSION 3.5)
project(robot_emu)

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
find_package(std_msgs REQUIRED)
find_package(robot_descriptions REQUIRED)

add_library(
    ros_node
    src/node.cpp
)

target_include_directories(
    ros_node
    PUBLIC
    include
)

ament_target_dependencies(ros_node rclcpp std_msgs robot_descriptions)

add_executable(robot_emu src/robot-emu.cpp)

ament_target_dependencies(robot_emu rclcpp)

target_link_libraries(robot_emu ros_node)

install(
  TARGETS robot_emu
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()