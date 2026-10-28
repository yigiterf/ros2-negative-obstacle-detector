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

#include "negative_obstacle_detector/free_space_detector.hpp"

#include <algorithm>
#include <cmath>

namespace negative_obstacle_detector
{

namespace
{

/// Forward fan lower bound [deg]. The detector only reasons about the ground
/// directly in front of the robot, where a pit would be traversed.
constexpr double kMinAngleDeg = -40.0;
/// Forward fan upper bound [deg].
constexpr double kMaxAngleDeg = 40.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

}  // namespace

FreeSpaceDetector::FreeSpaceDetector(const FreeSpaceDetectorParams & params)
: params_(params)
{
  const double span = kMaxAngleDeg - kMinAngleDeg;
  num_bins_ = static_cast<int>(std::ceil(span / params_.angle_bin_size));
  if (num_bins_ < 1) {
    num_bins_ = 1;
  }
  bins_.resize(num_bins_);
}

void FreeSpaceDetector::detect(
  const pcl::PointCloud<PointT>::ConstPtr & ground,
  const GroundPlane & plane,
  pcl::PointCloud<PointT> & leading_edges)
{
  for (auto & bin : bins_) {
    bin.clear();
  }

  // Bin ground returns by polar angle relative to the sensor.
  for (const auto & point : ground->points) {
    const double dx = static_cast<double>(point.x) - params_.sensor_x;
    const double dy = static_cast<double>(point.y) - params_.sensor_y;
    const double range = std::hypot(dx, dy);
    if (range < params_.min_range || range > params_.max_range) {
      continue;
    }

    const double angle_deg = std::atan2(dy, dx) / kDegToRad;
    if (angle_deg < kMinAngleDeg || angle_deg > kMaxAngleDeg) {
      continue;
    }

    int index = static_cast<int>(
      std::floor((angle_deg - kMinAngleDeg) / params_.angle_bin_size));
    if (index < 0) {
      index = 0;
    } else if (index >= num_bins_) {
      index = num_bins_ - 1;
    }
    bins_[index].push_back(range);
  }

  leading_edges.clear();
  leading_edges.reserve(static_cast<std::size_t>(num_bins_));

  // A degenerate plane cannot provide a ground height; nothing to emit.
  if (std::abs(plane.c) < 1e-6F) {
    return;
  }
  const float ground_z = -plane.d / plane.c;

  for (int i = 0; i < num_bins_; ++i) {
    std::vector<double> & bin = bins_[i];
    if (bin.size() < 2) {
      continue;
    }

    std::sort(bin.begin(), bin.end());

    const double center_deg =
      kMinAngleDeg + (static_cast<double>(i) + 0.5) * params_.angle_bin_size;
    const double center_rad = center_deg * kDegToRad;

    for (std::size_t j = 1; j < bin.size(); ++j) {
      if (bin[j] - bin[j - 1] <= params_.gap_threshold) {
        continue;
      }

      // The smaller range is the leading edge of the negative obstacle.
      const double r = bin[j - 1];
      PointT edge;
      edge.x = static_cast<float>(params_.sensor_x + r * std::cos(center_rad));
      edge.y = static_cast<float>(params_.sensor_y + r * std::sin(center_rad));
      edge.z = ground_z;
      leading_edges.push_back(edge);
    }
  }
}

}  // namespace negative_obstacle_detector
