// Copyright 2026 negative_obstacle_detector contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEGATIVE_OBSTACLE_DETECTOR__DETECTOR_NODE_HPP_
#define NEGATIVE_OBSTACLE_DETECTOR__DETECTOR_NODE_HPP_

#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "negative_obstacle_detector/cloud_preprocessor.hpp"
#include "negative_obstacle_detector/free_space_detector.hpp"
#include "negative_obstacle_detector/ground_segmenter.hpp"
#include "negative_obstacle_detector/negative_obstacle_extractor.hpp"

namespace negative_obstacle_detector
{

/// Lifecycle node that runs the perception pipeline.
///
/// Pipeline: TF transform -> preprocess -> ground segmentation ->
/// negative/elevated obstacle extraction -> publish.
class DetectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit DetectorNode(const rclcpp::NodeOptions & options);
  ~DetectorNode() override = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void on_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::string input_topic_;
  std::string target_frame_;
  std::string output_topic_;
  std::string ground_topic_;
  std::string negative_topic_;
  std::string elevated_topic_;
  std::string hazards_topic_;
  std::string markers_topic_;

  std::unique_ptr<CloudPreprocessor> preprocessor_;
  std::unique_ptr<GroundSegmenter> ground_segmenter_;
  std::unique_ptr<NegativeObstacleExtractor> extractor_;
  std::unique_ptr<FreeSpaceDetector> free_space_detector_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    preprocessed_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    ground_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    negative_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    elevated_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    hazards_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    markers_pub_;

  // Reused PCL buffers (avoid per-frame allocation).
  pcl::PointCloud<PointT>::Ptr input_cloud_;
  pcl::PointCloud<PointT>::Ptr preprocessed_cloud_;
  pcl::PointCloud<PointT>::Ptr ground_cloud_;
  pcl::PointCloud<PointT>::Ptr non_ground_cloud_;
  pcl::PointCloud<PointT>::Ptr negative_cloud_;
  pcl::PointCloud<PointT>::Ptr elevated_cloud_;
  pcl::PointCloud<PointT>::Ptr leading_edges_;
  pcl::PointCloud<PointT>::Ptr hazards_cloud_;
};

}  // namespace negative_obstacle_detector

#endif  // NEGATIVE_OBSTACLE_DETECTOR__DETECTOR_NODE_HPP_
