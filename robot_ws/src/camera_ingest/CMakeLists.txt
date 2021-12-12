cmake_minimum_required(VERSION 3.5)
project(camera_ingest)

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
#find_package(image_transport REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(compressed_image_transport REQUIRED)
find_package(CUDA REQUIRED)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

add_subdirectory(ros_node)

set(gxi_LIBRARY ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/lib/gxilib/libgxiapi.so)
set(gxi_GVTL ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/lib/gxilib/GxGVTL.cti)
set(gxi_U3VTL ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/lib/gxilib/GxU3VTL.cti)
set(gxi_INCLUDES ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/include/gxilib)

add_library(gxiapi SHARED IMPORTED)
set_target_properties(gxiapi PROPERTIES IMPORTED_LOCATION ${gxi_LIBRARY} ${gxi_GVTL} ${gxi_U3VTL})

set(ndi_LIBRARY ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/lib/libndi/libndi.so)
set(ndi5_LIBRARY ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/lib/libndi/libndi.so.5)
set(ndi_INCLUDES ${PROJECT_SOURCE_DIR}/../../../NSSC_private_libs/include/libndi)

add_library(libndi SHARED IMPORTED)
set_target_properties(libndi PROPERTIES IMPORTED_LOCATION ${ndi5_LIBRARY})

set(jutils_LIBRARY ${PROJECT_SOURCE_DIR}/../../../aarch64/lib/libjetson-utils.so)
set(jinference_LIBRARY ${PROJECT_SOURCE_DIR}/../../../aarch64/lib/libjetson-inference.so)
set(jetson_INCLUDES ${PROJECT_SOURCE_DIR}/../../../aarch64/include)

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

add_library(
  nssc_errors
  src/errors/nssc_errors.cpp
)

target_include_directories(
  nssc_errors
  PUBLIC
  include/errors
  ${gxi_INCLUDES}
)

target_link_libraries(nssc_errors ros_node gxiapi)

ament_target_dependencies(nssc_errors rclcpp)

add_library(
  frame
  src/frame/mono_frame.cpp
  src/frame/stereo_frame.cpp
)

target_include_directories(
  frame
  PUBLIC
  include/frame
  include/frame/frame_struct
  include/errors
  ${gxi_INCLUDES}
  ${CUDA_INCLUDE_DIRS}
  ${jetson_INCLUDES}
)

target_link_libraries(frame ros_node gxiapi jutils jinference NPP NPPI)

ament_target_dependencies(frame OpenCV CUDA rclcpp)

add_library(
  camera
  src/camera/camera.cpp
  src/camera/load_camera.cpp
)

target_include_directories(
  camera
  PUBLIC
  include/camera
  include/errors
  include/concurrentqueue
  include/frame/frame_struct
  include/camera_manager/cam_sync
  ${gxi_INCLUDES}
  ${CUDA_INCLUDE_DIRS}
  ${jetson_INCLUDES}
)

target_link_libraries(camera frame ros_node gxiapi jutils jinference)

ament_target_dependencies(camera OpenCV CUDA rclcpp)

add_library(
  camera_manager
  src/camera_manager/camera_manager.cpp
)

target_include_directories(
  camera_manager
  PUBLIC
  include/errors
  include/camera_manager
  include/camera_manager/cam_sync
  include/camera
  include/frame/frame_struct
)

target_link_libraries(camera_manager ros_node camera)

ament_target_dependencies(camera_manager rclcpp)

add_library(
  ndi_send
  src/ndi/ndi.cpp
)

target_include_directories(
  ndi_send
  PUBLIC
  include/ndi
  include/errors
  include/camera_manager
  include/frame/frame_struct
  ${ndi_INCLUDES}
  ${CUDA_INCLUDE_DIRS}
  ${jetson_INCLUDES}
)

target_link_libraries(ndi_send ros_node camera_manager libndi jutils jinference)

ament_target_dependencies(ndi_send rclcpp OpenCV CUDA)

#add_library(
#  nssc_image_transport
#  src/image_transport/image_transport.cpp
#)

#target_include_directories(
#  nssc_image_transport
#  PUBLIC
#  include/ndi
#  include/errors
#  include/image_transport
#  include/camera_manager
#  include/concurrentqueue
#)

#target_link_libraries(nssc_image_transport ros_node camera_manager ndi_send)

#ament_target_dependencies(nssc_image_transport rclcpp image_transport compressed_image_transport OpenCV cv_bridge)

add_executable(main_node src/main_node.cpp)

ament_target_dependencies(main_node rclcpp)

target_link_libraries(main_node ros_node camera ndi_send nssc_errors camera_manager libndi gxiapi)

install(
  TARGETS camera ndi_send nssc_errors camera_manager #nssc_image_transport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES ${gxi_LIBRARY} ${gxi_GVTL} ${gxi_U3VTL} ${ndi_LIBRARY} ${ndi5_LIBRARY} ${jinference_LIBRARY} ${jutils_LIBRARY}
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()