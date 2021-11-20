find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_descriptions REQUIRED)

add_library(
  nsra2
  SHARED
  src/nsra2_system_position.cpp
)

target_include_directories(
  nsra2
  PRIVATE
  include
)

ament_target_dependencies(
  nsra2
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  robot_descriptions
)