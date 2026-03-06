cmake_minimum_required(VERSION 3.8)
project(deepstream_tracker)

# --- ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# --- GStreamer ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

include_directories(
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  /opt/nvidia/deepstream/deepstream/sources/includes
)

link_directories(
  ${GST_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
)

# --- DeepStream libraries ---
find_library(NVDSGST_META_LIB nvdsgst_meta PATHS /opt/nvidia/deepstream/deepstream/lib REQUIRED)
find_library(NVDS_META_LIB nvds_meta PATHS /opt/nvidia/deepstream/deepstream/lib REQUIRED)
find_library(NVDS_INFER_LIB nvds_infer PATHS /opt/nvidia/deepstream/deepstream/lib REQUIRED)
find_library(NVDS_INFERUTILS_LIB nvds_inferutils PATHS /opt/nvidia/deepstream/deepstream/lib REQUIRED)
find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64 REQUIRED)

# --- Build executable ---
add_executable(deepstream_tracker_node
  src/deepstream_tracker_node.cpp
)

ament_target_dependencies(deepstream_tracker_node
  rclcpp 
  sensor_msgs 
  vision_msgs
  image_transport
  cv_bridge
)

target_link_libraries(deepstream_tracker_node
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${NVDSGST_META_LIB}
  ${NVDS_META_LIB}
  ${NVDS_INFER_LIB}
  ${NVDS_INFERUTILS_LIB}
  ${CUDART_LIB}
)

# --- Install rpath for runtime linking ---
set_target_properties(deepstream_tracker_node PROPERTIES
  INSTALL_RPATH "/opt/nvidia/deepstream/deepstream/lib;/usr/local/cuda/lib64"
)

# --- Install targets ---
install(TARGETS deepstream_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
