cmake_minimum_required(VERSION 3.8)
project(camera_bringup_ros2)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/camera_stream_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
