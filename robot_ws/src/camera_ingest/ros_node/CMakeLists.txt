find_package(rclcpp REQUIRED)
find_package(fastcdr REQUIRED)
find_package(foonathan_memory REQUIRED)
find_package(fastrtps REQUIRED)

set(ndi_INCLUDES ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/include/libndi)
set(readline_INCLUDES /usr/include/readline)

add_library(
    ros_node
    src/node.cpp
    src/cli.cpp
)

target_include_directories(
    ros_node
    PUBLIC
    include
    ${ndi_INCLUDES}
    ${readline_INCLUDES}
)

ament_target_dependencies(ros_node rclcpp)

target_link_libraries(ros_node fastrtps fastcdr foonathan_memory Readline)