cmake_minimum_required(VERSION 3.8)
project(detections_img)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detections_img_node src/main.cpp)

ament_target_dependencies(
  detections_img_node
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)

install(TARGETS detections_img_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
