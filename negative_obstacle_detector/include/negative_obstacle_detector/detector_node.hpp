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

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "negative_obstacle_detector/cloud_preprocessor.hpp"

namespace negative_obstacle_detector
{

/// Lifecycle node that runs the perception pipeline.
///
/// Phase 2 scope: subscribe to a `PointCloud2` topic, preprocess it and
/// publish the result. Ground segmentation (Phase 3) and costmap projection
/// (Phase 4) plug into the same node.
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

  std::unique_ptr<CloudPreprocessor> preprocessor_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    preprocessed_pub_;

  // Reused PCL buffers (avoid per-frame allocation).
  pcl::PointCloud<PointT>::Ptr input_cloud_;
  pcl::PointCloud<PointT> output_cloud_;

  std::string input_topic_;
  std::string output_topic_;
};

}  // namespace negative_obstacle_detector

#endif  // NEGATIVE_OBSTACLE_DETECTOR__DETECTOR_NODE_HPP_
