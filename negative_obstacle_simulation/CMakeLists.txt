cmake_minimum_required(VERSION 3.16)
project(negative_obstacle_simulation NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
