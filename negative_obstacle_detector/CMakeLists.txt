cmake_minimum_required(VERSION 3.16)
project(negative_obstacle_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL 1.14 REQUIRED COMPONENTS common filters segmentation)

add_library(negative_obstacle_detector_core
  src/cloud_preprocessor.cpp
  src/ground_segmenter.cpp
  src/negative_obstacle_extractor.cpp
  src/free_space_detector.cpp
)
target_include_directories(negative_obstacle_detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(negative_obstacle_detector_core PUBLIC
  ${PCL_LIBRARIES}
)
target_include_directories(negative_obstacle_detector_core SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(negative_obstacle_detector_core PUBLIC
  ${PCL_DEFINITIONS}
)

add_executable(negative_obstacle_detector_node
  src/detector_node.cpp
  src/main.cpp
)
target_link_libraries(negative_obstacle_detector_node
  negative_obstacle_detector_core
)
ament_target_dependencies(negative_obstacle_detector_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  visualization_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
)

install(
  DIRECTORY include/ config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS negative_obstacle_detector_core negative_obstacle_detector_node
  EXPORT ${PROJECT_NAME}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle sensor_msgs visualization_msgs pcl_conversions tf2_ros tf2_eigen tf2_sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_cloud_preprocessor
    test/test_cloud_preprocessor.cpp)
  target_link_libraries(test_cloud_preprocessor negative_obstacle_detector_core)

  ament_add_gtest(test_ground_segmenter
    test/test_ground_segmenter.cpp)
  target_link_libraries(test_ground_segmenter negative_obstacle_detector_core)

  ament_add_gtest(test_negative_obstacle_extractor
    test/test_negative_obstacle_extractor.cpp)
  target_link_libraries(test_negative_obstacle_extractor negative_obstacle_detector_core)

  ament_add_gtest(test_free_space_detector
    test/test_free_space_detector.cpp)
  target_link_libraries(test_free_space_detector negative_obstacle_detector_core)
endif()

ament_package()
