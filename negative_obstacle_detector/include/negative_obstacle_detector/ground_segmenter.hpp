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

#ifndef NEGATIVE_OBSTACLE_DETECTOR__GROUND_SEGMENTER_HPP_
#define NEGATIVE_OBSTACLE_DETECTOR__GROUND_SEGMENTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace negative_obstacle_detector
{

using PointT = pcl::PointXYZ;

/// A plane expressed as `a*x + b*y + c*z + d = 0` with a unit normal.
struct GroundPlane
{
  float a = 0.0F;
  float b = 0.0F;
  float c = 1.0F;
  float d = 0.0F;
  bool valid = false;

  /// Signed distance of a point to the plane. Negative = below the plane.
  float signed_distance(const PointT & point) const
  {
    return a * point.x + b * point.y + c * point.z + d;
  }
};

/// Parameters for RANSAC ground-plane estimation.
struct GroundSegmenterParams
{
  /// Maximum distance from a point to the plane to count as an inlier [m].
  double distance_threshold = 0.03;
  /// Maximum angle between the plane normal and the gravity axis [rad].
  double angle_tolerance = 0.26;  // ~15 deg
  /// RANSAC iteration budget.
  int max_iterations = 200;
};

/// Estimates the ground plane of a gravity-aligned cloud.
///
/// Uses `SACMODEL_PERPENDICULAR_PLANE` with the gravity axis (0, 0, 1): the
/// plane normal is forced to stay within `angle_tolerance` of vertical. This
/// prevents a wall or table top from being accepted as "ground". The input
/// cloud must already be expressed in a gravity-aligned frame (e.g. base_link).
class GroundSegmenter
{
public:
  explicit GroundSegmenter(const GroundSegmenterParams & params);
  ~GroundSegmenter() = default;

  GroundSegmenter(const GroundSegmenter &) = delete;
  GroundSegmenter & operator=(const GroundSegmenter &) = delete;

  /// Splits `input` into `ground` and `non_ground`. Returns false if no valid
  /// horizontal plane was found, in which case the outputs are untouched.
  bool segment(
    const pcl::PointCloud<PointT>::ConstPtr & input,
    pcl::PointCloud<PointT> & ground,
    pcl::PointCloud<PointT> & non_ground);

  /// The last estimated ground plane.
  const GroundPlane & plane() const { return plane_; }

private:
  GroundSegmenterParams params_;
  GroundPlane plane_;

  pcl::SACSegmentation<PointT> seg_;
  pcl::PointIndices::Ptr inliers_;
  pcl::ModelCoefficients::Ptr coefficients_;
};

}  // namespace negative_obstacle_detector

#endif  // NEGATIVE_OBSTACLE_DETECTOR__GROUND_SEGMENTER_HPP_
