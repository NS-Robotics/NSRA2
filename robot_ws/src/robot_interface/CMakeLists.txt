cmake_minimum_required(VERSION 3.5)
project(robot_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)

find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
#find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        rclcpp
        rclcpp_action
        tf2_geometry_msgs
        tf2_ros
        moveit_core
        rviz_visual_tools
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        moveit_ros_planning
        pluginlib
        Eigen3
        Boost
        control_msgs
        moveit_servo
        )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_executable(nsra2_interface nsra2/src/nsra2_interface.cpp)

ament_target_dependencies(nsra2_interface
        ament_cmake
        rclcpp
        rclcpp_action
)

ament_target_dependencies(
        nsra2_interface
        tf2_geometry_msgs
        tf2_ros
        moveit_core
        rviz_visual_tools
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        moveit_ros_planning
        pluginlib
        Eigen3
        Boost
        control_msgs
        moveit_servo
        Boost
)

install(
        TARGETS nsra2_interface
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY nsra2/include/
        DESTINATION include
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)

ament_package()
