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

#ifndef NEGATIVE_OBSTACLE_DETECTOR__FREE_SPACE_DETECTOR_HPP_
#define NEGATIVE_OBSTACLE_DETECTOR__FREE_SPACE_DETECTOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "negative_obstacle_detector/ground_segmenter.hpp"

namespace negative_obstacle_detector
{

/// Tunable parameters for the free-space (missing-ground) detector.
struct FreeSpaceDetectorParams
{
  /// Angular width of each polar bin [deg].
  double angle_bin_size = 1.0;
  /// Minimum range jump between two consecutive ground returns that indicates
  /// a negative obstacle (the far wall of a pit has no ground coverage) [m].
  double gap_threshold = 0.3;
  /// Minimum radial distance from the sensor that is considered [m].
  double min_range = 0.4;
  /// Maximum radial distance from the sensor that is considered [m].
  double max_range = 6.0;
  /// Sensor origin in the gravity-aligned frame [m].
  double sensor_x = 0.25;
  /// Sensor origin in the gravity-aligned frame [m].
  double sensor_y = 0.0;
  /// Sensor origin in the gravity-aligned frame [m] (informative).
  double sensor_z = 0.30;
};

/// Detects the leading edge of a negative obstacle from a gap in ground coverage.
///
/// Negative obstacles are only partially visible: the far wall of a pit returns
/// points *below* the ground plane (handled by `NegativeObstacleExtractor`), but
/// the pit floor itself returns nothing, leaving a radial gap in the ground
/// cloud right at the leading edge. This detector finds that gap:
///
///   1. project each ground point to polar coordinates (angle, range) relative
///      to the sensor origin;
///   2. keep points within the forward fan and range window;
///   3. bin points by angle (1 deg bins) and sort each bin by range;
///   4. where two consecutive ranges differ by more than `gap_threshold`, the
///      smaller range is the leading edge of a negative obstacle;
///   5. emit a synthetic point at that range on the ground plane.
///
/// All working buffers are owned by the instance and reused across calls, so
/// steady-state processing does not allocate on the heap per frame.
class FreeSpaceDetector
{
public:
  explicit FreeSpaceDetector(const FreeSpaceDetectorParams & params);
  ~FreeSpaceDetector() = default;

  FreeSpaceDetector(const FreeSpaceDetector &) = delete;
  FreeSpaceDetector & operator=(const FreeSpaceDetector &) = delete;

  /// Detects leading edges in `ground` and appends them to `leading_edges`.
  ///
  /// `leading_edges` is cleared first; the caller may pass the same cloud
  /// instance across frames. `plane` supplies the ground height
  /// (z = -d / c for the plane a*x + b*y + c*z + d = 0).
  void detect(
    const pcl::PointCloud<PointT>::ConstPtr & ground,
    const GroundPlane & plane,
    pcl::PointCloud<PointT> & leading_edges);

private:
  FreeSpaceDetectorParams params_;

  /// Number of angular bins covering the forward fan.
  int num_bins_ = 0;

  /// Per-bin ranges, reused across calls to avoid per-frame allocation.
  std::vector<std::vector<double>> bins_;
};

}  // namespace negative_obstacle_detector

#endif  // NEGATIVE_OBSTACLE_DETECTOR__FREE_SPACE_DETECTOR_HPP_
