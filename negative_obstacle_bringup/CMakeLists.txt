cmake_minimum_required(VERSION 3.16)
project(negative_obstacle_bringup NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
