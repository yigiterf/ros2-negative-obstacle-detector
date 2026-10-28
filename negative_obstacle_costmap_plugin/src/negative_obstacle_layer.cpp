// Copyright 2026 negative_obstacle_costmap_plugin contributors.
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

#include "negative_obstacle_costmap_plugin/negative_obstacle_layer.hpp"

#include <functional>
#include <memory>
#include <utility>

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

PLUGINLIB_EXPORT_CLASS(
  negative_obstacle_costmap_plugin::NegativeObstacleLayer, nav2_costmap_2d::Layer)

namespace negative_obstacle_costmap_plugin
{

void NegativeObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("NegativeObstacleLayer: costmap node is not available");
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();
  hazards_topic_ = node->declare_parameter(
    getFullName("hazards_topic"), std::string("/negative_obstacle/hazards"));
  node->get_parameter(getFullName("hazards_topic"), hazards_topic_);
  enabled_ = node->declare_parameter(getFullName("enabled"), true);

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    hazards_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NegativeObstacleLayer::on_point_cloud, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(logger_, "NegativeObstacleLayer: subscribing to %s (frame %s)",
    hazards_topic_.c_str(), global_frame_.c_str());
}

void NegativeObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  hazard_points_.clear();
}

void NegativeObstacleLayer::on_point_cloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 transformed;
  try {
    transformed = tf_->transform(*msg, global_frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "NegativeObstacleLayer: transform to %s failed: %s",
      global_frame_.c_str(), ex.what());
    return;
  }

  std::vector<std::array<double, 2>> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed, "y");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    points.push_back({*iter_x, *iter_y});
  }

  std::lock_guard<std::mutex> lock(mutex_);
  hazard_points_.swap(points);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 3000,
    "NegativeObstacleLayer: received %zu hazard points", hazard_points_.size());
}

void NegativeObstacleLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & point : hazard_points_) {
    *min_x = std::min(*min_x, point[0]);
    *min_y = std::min(*min_y, point[1]);
    *max_x = std::max(*max_x, point[0]);
    *max_y = std::max(*max_y, point[1]);
  }
}

void NegativeObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int, int, int, int)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & point : hazard_points_) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (master_grid.worldToMap(point[0], point[1], mx, my)) {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

}  // namespace negative_obstacle_costmap_plugin
