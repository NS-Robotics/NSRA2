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

#add_subdirectory(nsra2)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_descriptions REQUIRED)

set(serial_INCLUDES /usr/local/include/serial)
set(serial_LIRBARY /usr/local/lib/libserial.so)

add_library(serial SHARED IMPORTED)
set_target_properties(serial PROPERTIES IMPORTED_LOCATION ${serial_LIRBARY})

add_library(
        nsra2_control
        nsra2/src/nsra2_control/nsra2_control.cpp
)

target_include_directories(
        nsra2_control
        PUBLIC
        nsra2/include/nsra2_control
        ${serial_INCLUDES}
)

target_link_libraries(
        nsra2_control
        serial
)

ament_target_dependencies(
        nsra2_control
        rclcpp
)

set_property(TARGET nsra2_control PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(
        nsra2_hardware
        SHARED
        nsra2/src/nsra2_system_position.cpp
)

target_include_directories(
        nsra2_hardware
        PRIVATE
        nsra2/include
)

target_link_libraries(
        nsra2_hardware
        nsra2_control
)

ament_target_dependencies(
        nsra2_hardware
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        robot_descriptions
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# INSTALL
install(
  TARGETS nsra2_hardware
  DESTINATION lib
)

install(
  DIRECTORY nsra2/include/ ${serial_INCLUDES}
  DESTINATION include
)

install(
        FILES ${serial_LIRBARY}
        DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
        nsra2/include
)

ament_export_libraries(
        nsra2_hardware
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()