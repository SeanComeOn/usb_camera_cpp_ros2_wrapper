cmake_minimum_required(VERSION 3.8)
project(bi_camera_cpp_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_support
  src/camera_support.cpp
)
target_include_directories(${PROJECT_NAME}_support PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}_support
  yaml-cpp
)

add_executable(camera_config_wizard
  src/camera_config_wizard.cpp
)
target_link_libraries(camera_config_wizard
  ${PROJECT_NAME}_support
)

add_executable(stereo_camera_node
  src/stereo_camera_node.cpp
)
target_link_libraries(stereo_camera_node
  ${PROJECT_NAME}_support
  ${OpenCV_LIBS}
)
ament_target_dependencies(stereo_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(camera_fps_benchmark
  src/camera_fps_benchmark.cpp
)
target_link_libraries(camera_fps_benchmark
  ${PROJECT_NAME}_support
  ${OpenCV_LIBS}
)
ament_target_dependencies(camera_fps_benchmark
  rclcpp
)

install(TARGETS
  camera_config_wizard
  stereo_camera_node
  camera_fps_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
