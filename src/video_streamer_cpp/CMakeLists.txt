cmake_minimum_required(VERSION 3.8)
project(video_streamer_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add your executable
add_executable(video_streamer_node src/main.cpp)

# Link the dependencies
ament_target_dependencies(video_streamer_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# **Add this line**
target_link_libraries(video_streamer_node ${OpenCV_LIBS})

# Install the executable
install(TARGETS
  video_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
