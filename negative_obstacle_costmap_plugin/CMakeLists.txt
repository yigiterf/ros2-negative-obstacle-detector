cmake_minimum_required(VERSION 3.16)
project(negative_obstacle_costmap_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(negative_obstacle_layer SHARED
  src/negative_obstacle_layer.cpp
)
target_include_directories(negative_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(negative_obstacle_layer
  nav2_costmap_2d
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(
  TARGETS negative_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(negative_obstacle_layer)
ament_export_dependencies(nav2_costmap_2d pluginlib)

ament_package()
