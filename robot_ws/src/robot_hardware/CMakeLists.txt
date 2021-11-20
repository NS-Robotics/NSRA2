cmake_minimum_required(VERSION 3.5)
project(robot_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_subdirectory(nsra2)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# INSTALL
install(
  TARGETS nsra2
  DESTINATION lib
)

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
#ament_export_include_directories(
#  include
#)

ament_export_libraries(
  nsra2
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()