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

#include "negative_obstacle_detector/detector_node.hpp"

#include <chrono>
#include <functional>

#include <pcl_conversions/pcl_conversions.h>

namespace negative_obstacle_detector
{

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("negative_obstacle_detector", options),
  input_cloud_(new pcl::PointCloud<PointT>())
{
}

DetectorNode::CallbackReturn DetectorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  input_topic_ = declare_parameter<std::string>(
    "input_topic", "/depth_camera/points");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/negative_obstacle/preprocessed");

  PreprocessorParams params;
  params.min_range = declare_parameter<double>("preprocessor.min_range", 0.4);
  params.max_range = declare_parameter<double>("preprocessor.max_range", 6.0);
  params.min_height = declare_parameter<double>("preprocessor.min_height", -0.6);
  params.max_height = declare_parameter<double>("preprocessor.max_height", 1.2);
  params.half_width = declare_parameter<double>("preprocessor.half_width", 3.0);
  params.voxel_leaf_size =
    declare_parameter<double>("preprocessor.voxel_leaf_size", 0.02);

  preprocessor_ = std::make_unique<CloudPreprocessor>(params);

  preprocessed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "Configured: input=%s output=%s",
    input_topic_.c_str(), output_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::on_point_cloud, this, std::placeholders::_1));
  preprocessed_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  cloud_sub_.reset();
  preprocessed_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  cloud_sub_.reset();
  preprocessed_pub_.reset();
  preprocessor_.reset();
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void DetectorNode::on_point_cloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto start = std::chrono::steady_clock::now();

  // ROS -> PCL. This is the single conversion boundary on the input side.
  pcl::fromROSMsg(*msg, *input_cloud_);

  preprocessor_->process(input_cloud_, output_cloud_);

  // PCL -> ROS on the output side.
  auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(output_cloud_, *out_msg);
  out_msg->header = msg->header;
  preprocessed_pub_->publish(std::move(out_msg));

  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start).count();

  RCLCPP_DEBUG(get_logger(),
    "Preprocessed %zu points in %ld us", output_cloud_.size(), elapsed);
}

}  // namespace negative_obstacle_detector
