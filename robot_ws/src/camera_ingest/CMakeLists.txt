cmake_minimum_required(VERSION 3.5)
project(camera_ingest)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(image_transport REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(compressed_image_transport REQUIRED)
find_package(CUDA REQUIRED)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ColorFilterParams.msg"
  "msg/CameraSettings.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(ros_node)

set(private_LIBRARIES /home/ros2/NSSC_private_libs)

set(gxi_LIBRARY ${private_LIBRARIES}/lib/gxilib/libgxiapi.so)
set(gxi_GVTL ${private_LIBRARIES}/lib/gxilib/GxGVTL.cti)
set(gxi_U3VTL ${private_LIBRARIES}/lib/gxilib/GxU3VTL.cti)
set(gxi_INCLUDES ${private_LIBRARIES}/include/gxilib)

add_library(gxiapi SHARED IMPORTED)
set_target_properties(gxiapi PROPERTIES IMPORTED_LOCATION ${gxi_LIBRARY} ${gxi_GVTL} ${gxi_U3VTL})

set(ndi_LIBRARY ${private_LIBRARIES}/lib/libndi/libndi.so)
set(ndi5_LIBRARY ${private_LIBRARIES}/lib/libndi/libndi.so.5)
set(ndi_INCLUDES ${private_LIBRARIES}/include/libndi)

add_library(libndi SHARED IMPORTED)
set_target_properties(libndi PROPERTIES IMPORTED_LOCATION ${ndi5_LIBRARY})

set(jutils_LIBRARY ${private_LIBRARIES}/aarch64/lib/libjetson-utils.so)
set(jinference_LIBRARY ${private_LIBRARIES}/aarch64/lib/libjetson-inference.so)
set(jetson_INCLUDES ${private_LIBRARIES}/aarch64/include)

add_library(jinference SHARED IMPORTED)
set_target_properties(jinference PROPERTIES IMPORTED_LOCATION ${jinference_LIBRARY})

add_library(jutils SHARED IMPORTED)
set_target_properties(jutils PROPERTIES IMPORTED_LOCATION ${jutils_LIBRARY})

set(npp_LIBRARY /usr/local/cuda/lib64/libnppc.so)
set(nppi_LIBRARY /usr/local/cuda/lib64/libnppicc.so)

add_library(NPPI SHARED IMPORTED)
add_library(NPP SHARED IMPORTED)
set_target_properties(NPP PROPERTIES IMPORTED_LOCATION ${npp_LIBRARY})
set_target_properties(NPPI PROPERTIES IMPORTED_LOCATION ${nppi_LIBRARY})

set(readline_INCLUDES /usr/include/readline)
set(readline_LIBRARY /usr/lib/aarch64-linux-gnu/libreadline.so)
add_library(readline SHARED IMPORTED)
set_target_properties(readline PROPERTIES IMPORTED_LOCATION ${readline_LIBRARY})

set(boost_INCLUDES /usr/include/boost)

add_library(
        nssc_errors
        src/application/errors/nssc_errors.cpp
)

target_include_directories(
        nssc_errors
        PUBLIC
        include/application/errors
        ${gxi_INCLUDES}
)

target_link_libraries(nssc_errors ros_node gxiapi)

ament_target_dependencies(nssc_errors rclcpp)

add_library(
        frame
        src/camera_api/frame/mono_frame.cpp
        src/camera_api/frame/stereo_frame.cpp
)

target_include_directories(
        frame
        PUBLIC
        include/camera_api/frame
        include/camera_api/frame/frame_struct
        include/application/errors
        ${gxi_INCLUDES}
        ${CUDA_INCLUDE_DIRS}
        ${jetson_INCLUDES}
)

target_link_libraries(frame ros_node gxiapi jutils jinference NPP NPPI)

ament_target_dependencies(frame OpenCV CUDA rclcpp)

add_library(
        camera
        src/camera_api/camera/camera.cpp
        src/camera_api/camera/load_camera.cpp
)

target_include_directories(
        camera
        PUBLIC
        include/camera_api/camera
        include/application/errors
        include/camera_api/concurrentqueue
        include/camera_api/frame/frame_struct
        include/camera_api/camera_manager/cam_sync
        ${gxi_INCLUDES}
        ${CUDA_INCLUDE_DIRS}
        ${jetson_INCLUDES}
)

target_link_libraries(camera frame ros_node gxiapi jutils jinference)

ament_target_dependencies(camera OpenCV CUDA rclcpp)

add_library(
        camera_manager
        src/camera_api/camera_manager/camera_manager.cpp
)

target_include_directories(
        camera_manager
        PUBLIC
        include/application/errors
        include/camera_api/camera_manager
        include/camera_api/camera_manager/cam_sync
        include/camera_api/camera
        include/camera_api/frame/frame_struct
)

target_link_libraries(camera_manager ros_node camera)

ament_target_dependencies(camera_manager rclcpp)

add_library(
        frame_manager
        src/viewer_api/frame_manager/frame_manager.cpp
)

target_include_directories(
        frame_manager
        PUBLIC
        include/viewer_api/frame_manager
        include/application/errors
        include/camera_api/frame/frame_struct
)

target_link_libraries(frame_manager camera_manager ros_node)

ament_target_dependencies(frame_manager OpenCV rclcpp)

add_library(
        ndi_send
        src/viewer_api/ndi/ndi.cpp
)

target_include_directories(
        ndi_send
        PUBLIC
        include/viewer_api/ndi
        include/application/errors
        include/camera_api/camera_manager
        include/camera_api/frame/frame_struct
        ${ndi_INCLUDES}
        ${CUDA_INCLUDE_DIRS}
        ${jetson_INCLUDES}
)

target_link_libraries(ndi_send ros_node camera_manager libndi jutils jinference frame_manager)

ament_target_dependencies(ndi_send rclcpp OpenCV CUDA)

add_library(
        ingest
        src/calib_cameras/ingest/ingest.cpp
)

target_include_directories(
        ingest
        PUBLIC
        include/calib_cameras/ingest
        include/application/errors
        include/camera_api/camera_manager
        include/camera_api/frame/frame_struct
        include/calib_cameras/rapidxml-1.13
)

target_link_libraries(ingest ros_node camera_manager frame_manager)

ament_target_dependencies(ingest rclcpp OpenCV)

add_library(
        calibration
        src/calib_cameras/calibration/calibration.cpp
        src/calib_cameras/calibration/prepare_data.cpp
)

target_include_directories(
        calibration
        PUBLIC
        include/calib_cameras/calibration
        include/application/errors
        include/calib_cameras/rapidxml-1.13
        ${gxi_INCLUDES}
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(calibration ros_node frame_manager)

ament_target_dependencies(calibration rclcpp OpenCV Boost)

add_library(
        triangulation
        src/obj_calc/triangulation/triangulation.cpp
        src/obj_calc/triangulation/triangulation_interface.cpp
        src/obj_calc/object_detection.cpp
)

target_include_directories(
        triangulation
        PUBLIC
        include/obj_calc/triangulation/
        include/obj_calc/
        include/application/errors
        include/camera_api/camera_manager
        include/camera_api/frame
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(triangulation ros_node camera_manager frame_manager)

ament_target_dependencies(triangulation rclcpp OpenCV Eigen3)

#add_library(
#  nssc_image_transport
#  src/image_transport/image_transport.cpp
#)

#target_include_directories(
#  nssc_image_transport
#  PUBLIC
#  include/ndi
#  include/application/errors
#  include/image_transport
#  include/camera_api/camera_manager
#  include/concurrentqueue
#)

#target_link_libraries(nssc_image_transport ros_node camera_manager ndi_send)

#ament_target_dependencies(nssc_image_transport rclcpp image_transport compressed_image_transport OpenCV cv_bridge)

add_library(
        task_executor
        src/application/task_executor/executor.cpp
        src/application/task_executor/calibration_api.cpp
        src/application/task_executor/ingest_api.cpp
        src/application/task_executor/process_api.cpp
        src/application/task_executor/send_api.cpp
)

target_include_directories(
        task_executor
        PUBLIC
        include/application/task_executor
        include/application/errors
        include/camera_api/camera_manager
        include/viewer_api/ndi
        include/calib_cameras/ingest
        include/calib_cameras/calibration
        include/obj_calc/triangulation
        ${gxi_INCLUDES}
)

target_link_libraries(task_executor ndi_send camera_manager ingest calibration triangulation)

ament_target_dependencies(task_executor rclcpp)


add_library(
        cli
        src/application/cli/cli.cpp
        src/application/cli/cli_args.cpp
)

target_include_directories(
        cli
        PUBLIC
        include/application/cli
        include/application/task_executor
        include/application/errors
        ${readline_INCLUDES}
)

target_link_libraries(cli readline ros_node task_executor nssc_errors)

ament_target_dependencies(cli rclcpp)


add_library(
        message_handler
        src/application/message_handler/message_handler.cpp
)

target_include_directories(
        message_handler
        PUBLIC
        include/application/message_handler
        include/application/task_executor
        include/application/errors
)

target_link_libraries(message_handler ros_node task_executor nssc_errors)

ament_target_dependencies(message_handler rclcpp)

rosidl_target_interfaces(message_handler
        ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_library(
        nssc_api
        src/application/api/nssc.cpp
)

target_include_directories(
        nssc_api
        PUBLIC
        include/application/api
        include/application/task_executor
        include/application/message_handler
        include/application/errors
        include/application/cli
)

target_link_libraries(nssc_api task_executor ros_node cli message_handler nssc_errors)

ament_target_dependencies(nssc_api rclcpp)


add_executable(main_node src/application/main.cpp)

ament_target_dependencies(main_node rclcpp rclcpp_action)

target_link_libraries(main_node nssc_api)

install(
        TARGETS camera ndi_send nssc_errors camera_manager task_executor ingest calibration nssc_api cli message_handler triangulation frame_manager frame #nssc_image_transport
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        FILES ${gxi_LIBRARY} ${gxi_GVTL} ${gxi_U3VTL} ${ndi_LIBRARY} ${ndi5_LIBRARY} ${jinference_LIBRARY} ${jutils_LIBRARY} #${readline_INCLUDES}
        DESTINATION lib
)

install(
        TARGETS main_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()