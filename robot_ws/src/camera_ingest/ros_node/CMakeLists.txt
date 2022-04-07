find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fastcdr REQUIRED)
find_package(foonathan_memory REQUIRED)
find_package(fastrtps REQUIRED)

set(private_LIBRARIES ${PROJECT_SOURCE_DIR}/../../../../NSSC_private_libs)
set(rclcpp_INCLUDES /opt/ros/foxy/install/rclcpp/include/rclcpp)

set(ndi_INCLUDES ${private_LIBRARIES}/include/libndi)

add_library(
    ros_node
    src/node.cpp
)

target_include_directories(
    ros_node
    PUBLIC
    include
    ${ndi_INCLUDES}
        ${rclcpp_INCLUDES}
)

ament_target_dependencies(ros_node rclcpp ament_index_cpp)

target_link_libraries(ros_node fastrtps fastcdr foonathan_memory)

install(
        TARGETS ros_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)