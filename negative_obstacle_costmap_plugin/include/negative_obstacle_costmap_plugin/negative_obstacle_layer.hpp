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

#ifndef NEGATIVE_OBSTACLE_COSTMAP_PLUGIN__NEGATIVE_OBSTACLE_LAYER_HPP_
#define NEGATIVE_OBSTACLE_COSTMAP_PLUGIN__NEGATIVE_OBSTACLE_LAYER_HPP_

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace negative_obstacle_costmap_plugin
{

/// Nav2 costmap layer that marks negative obstacles and overhangs as lethal.
///
/// Subscribes to the hazard point cloud published by `negative_obstacle_detector`,
/// transforms it into the costmap global frame, and marks the occupied cells
/// with `LETHAL_OBSTACLE`. This layer marks only; it never clears, so a hazard
/// persists in the costmap until the layer is reset.
class NegativeObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  NegativeObstacleLayer() = default;
  ~NegativeObstacleLayer() override = default;

  void onInitialize() override;
  void reset() override;
  bool isClearable() override { return true; }
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  void on_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::string global_frame_;
  std::string hazards_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::mutex mutex_;
  // Hazard points transformed into the global frame (x, y only).
  std::vector<std::array<double, 2>> hazard_points_;
};

}  // namespace negative_obstacle_costmap_plugin

#endif  // NEGATIVE_OBSTACLE_COSTMAP_PLUGIN__NEGATIVE_OBSTACLE_LAYER_HPP_
