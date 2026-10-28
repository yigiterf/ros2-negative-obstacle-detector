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

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>

#include <std_msgs/msg/header.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace negative_obstacle_detector
{

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("negative_obstacle_detector", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  input_cloud_(new pcl::PointCloud<PointT>()),
  preprocessed_cloud_(new pcl::PointCloud<PointT>()),
  ground_cloud_(new pcl::PointCloud<PointT>()),
  non_ground_cloud_(new pcl::PointCloud<PointT>()),
  negative_cloud_(new pcl::PointCloud<PointT>()),
  elevated_cloud_(new pcl::PointCloud<PointT>()),
  leading_edges_(new pcl::PointCloud<PointT>()),
  hazards_cloud_(new pcl::PointCloud<PointT>())
{
}

DetectorNode::CallbackReturn DetectorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  input_topic_ = declare_parameter<std::string>(
    "input_topic", "/depth_camera/points");
  target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/negative_obstacle/preprocessed");
  ground_topic_ = declare_parameter<std::string>(
    "ground_topic", "/negative_obstacle/ground");
  negative_topic_ = declare_parameter<std::string>(
    "negative_topic", "/negative_obstacle/negative");
  elevated_topic_ = declare_parameter<std::string>(
    "elevated_topic", "/negative_obstacle/elevated");
  hazards_topic_ = declare_parameter<std::string>(
    "hazards_topic", "/negative_obstacle/hazards");
  markers_topic_ = declare_parameter<std::string>(
    "markers_topic", "/negative_obstacle/markers");

  PreprocessorParams pre_params;
  pre_params.min_range = declare_parameter<double>("preprocessor.min_range", 0.4);
  pre_params.max_range = declare_parameter<double>("preprocessor.max_range", 6.0);
  pre_params.min_height = declare_parameter<double>("preprocessor.min_height", -0.6);
  pre_params.max_height = declare_parameter<double>("preprocessor.max_height", 1.2);
  pre_params.half_width = declare_parameter<double>("preprocessor.half_width", 3.0);
  pre_params.voxel_leaf_size =
    declare_parameter<double>("preprocessor.voxel_leaf_size", 0.02);

  GroundSegmenterParams ground_params;
  ground_params.distance_threshold =
    declare_parameter<double>("ground.distance_threshold", 0.03);
  ground_params.angle_tolerance =
    declare_parameter<double>("ground.angle_tolerance", 0.26);
  ground_params.max_iterations =
    declare_parameter<int>("ground.max_iterations", 200);

  NegativeObstacleExtractorParams extract_params;
  extract_params.min_depth =
    declare_parameter<double>("negative_obstacle.min_depth", 0.05);
  extract_params.overhang_min_height =
    declare_parameter<double>("negative_obstacle.overhang_min_height", 0.05);
  extract_params.overhang_max_height =
    declare_parameter<double>("negative_obstacle.overhang_max_height", 1.20);
  extract_params.cluster_tolerance =
    declare_parameter<double>("negative_obstacle.cluster_tolerance", 0.05);
  extract_params.min_cluster_size =
    declare_parameter<int>("negative_obstacle.min_cluster_size", 8);

  FreeSpaceDetectorParams free_space_params;
  free_space_params.angle_bin_size =
    declare_parameter<double>("free_space.angle_bin_size", 1.0);
  free_space_params.gap_threshold =
    declare_parameter<double>("free_space.gap_threshold", 0.3);
  free_space_params.min_range =
    declare_parameter<double>("free_space.min_range", 0.4);
  free_space_params.max_range =
    declare_parameter<double>("free_space.max_range", 6.0);
  free_space_params.sensor_x =
    declare_parameter<double>("free_space.sensor_x", 0.25);
  free_space_params.sensor_y =
    declare_parameter<double>("free_space.sensor_y", 0.0);
  free_space_params.sensor_z =
    declare_parameter<double>("free_space.sensor_z", 0.30);

  preprocessor_ = std::make_unique<CloudPreprocessor>(pre_params);
  ground_segmenter_ = std::make_unique<GroundSegmenter>(ground_params);
  extractor_ = std::make_unique<NegativeObstacleExtractor>(extract_params);
  free_space_detector_ = std::make_unique<FreeSpaceDetector>(free_space_params);

  preprocessed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::SensorDataQoS());
  ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    ground_topic_, rclcpp::SensorDataQoS());
  negative_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    negative_topic_, rclcpp::SensorDataQoS());
  elevated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    elevated_topic_, rclcpp::SensorDataQoS());
  hazards_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    hazards_topic_, rclcpp::SensorDataQoS());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    markers_topic_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(),
    "Configured: input=%s target_frame=%s", input_topic_.c_str(), target_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::on_point_cloud, this, std::placeholders::_1));
  preprocessed_pub_->on_activate();
  ground_pub_->on_activate();
  negative_pub_->on_activate();
  elevated_pub_->on_activate();
  hazards_pub_->on_activate();
  markers_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  cloud_sub_.reset();
  preprocessed_pub_->on_deactivate();
  ground_pub_->on_deactivate();
  negative_pub_->on_deactivate();
  elevated_pub_->on_deactivate();
  hazards_pub_->on_deactivate();
  markers_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

DetectorNode::CallbackReturn DetectorNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  cloud_sub_.reset();
  preprocessed_pub_.reset();
  ground_pub_.reset();
  negative_pub_.reset();
  elevated_pub_.reset();
  hazards_pub_.reset();
  markers_pub_.reset();
  preprocessor_.reset();
  ground_segmenter_.reset();
  extractor_.reset();
  free_space_detector_.reset();
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

  // Transform to a gravity-aligned frame. The ground plane is only meaningful
  // in a frame where "z is up"; the sensor frame is pitched downward.
  sensor_msgs::msg::PointCloud2 transformed;
  try {
    transformed =
      tf_buffer_.transform(*msg, target_frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF transform to %s failed: %s", target_frame_.c_str(), ex.what());
    return;
  }

  pcl::fromROSMsg(transformed, *input_cloud_);

  preprocessor_->process(input_cloud_, *preprocessed_cloud_);

  // Ground segmentation; skip extraction if no plane was found.
  if (!ground_segmenter_->segment(preprocessed_cloud_, *ground_cloud_, *non_ground_cloud_)) {
    RCLCPP_DEBUG(get_logger(), "No ground plane found");
    return;
  }

  extractor_->extract(
    non_ground_cloud_, ground_segmenter_->plane(), *negative_cloud_, *elevated_cloud_);

  // Leading-edge detection from gaps in ground coverage (the pit floor itself
  // returns nothing, so the far wall is invisible from above).
  free_space_detector_->detect(
    ground_cloud_, ground_segmenter_->plane(), *leading_edges_);

  // Publish all stages.
  auto publish_cloud = [this](const pcl::PointCloud<PointT> & cloud,
    const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const std_msgs::msg::Header & header)
    {
      auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(cloud, *out);
      out->header = header;
      out->header.frame_id = target_frame_;
      pub->publish(std::move(out));
    };

  publish_cloud(*preprocessed_cloud_, preprocessed_pub_, transformed.header);
  publish_cloud(*ground_cloud_, ground_pub_, transformed.header);
  publish_cloud(*negative_cloud_, negative_pub_, transformed.header);
  publish_cloud(*elevated_cloud_, elevated_pub_, transformed.header);

  // Combined hazard cloud for the costmap layer (negative + elevated + leading
  // edges).
  hazards_cloud_->clear();
  hazards_cloud_->reserve(
    negative_cloud_->size() + elevated_cloud_->size() + leading_edges_->size());
  *hazards_cloud_ += *negative_cloud_;
  *hazards_cloud_ += *elevated_cloud_;
  *hazards_cloud_ += *leading_edges_;
  publish_cloud(*hazards_cloud_, hazards_pub_, transformed.header);

  // Visualization markers.
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
  const auto add_points_marker =
    [&](int id, const pcl::PointCloud<PointT> & cloud, float r, float g, float b)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = transformed.header.stamp;
      marker.ns = "negative_obstacle";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0F;
      marker.points.reserve(cloud.size());
      for (const auto & point : cloud.points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
      }
      markers->markers.push_back(marker);
    };
  add_points_marker(0, *negative_cloud_, 1.0F, 0.1F, 0.1F);
  add_points_marker(1, *elevated_cloud_, 1.0F, 0.6F, 0.1F);
  add_points_marker(2, *leading_edges_, 0.1F, 0.7F, 1.0F);
  markers_pub_->publish(std::move(markers));

  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start).count();
  RCLCPP_DEBUG(get_logger(),
    "ground=%zu non_ground=%zu negative=%zu elevated=%zu leading_edges=%zu in %ld us",
    ground_cloud_->size(), non_ground_cloud_->size(),
    negative_cloud_->size(), elevated_cloud_->size(), leading_edges_->size(), elapsed);
}

}  // namespace negative_obstacle_detector
