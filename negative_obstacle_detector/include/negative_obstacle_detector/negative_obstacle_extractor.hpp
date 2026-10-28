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

#ifndef NEGATIVE_OBSTACLE_DETECTOR__NEGATIVE_OBSTACLE_EXTRACTOR_HPP_
#define NEGATIVE_OBSTACLE_DETECTOR__NEGATIVE_OBSTACLE_EXTRACTOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "negative_obstacle_detector/ground_segmenter.hpp"

namespace negative_obstacle_detector
{

/// Parameters for the negative-obstacle and overhang extractor.
struct NegativeObstacleExtractorParams
{
  /// Minimum depth below the ground plane to count as a negative obstacle [m].
  /// Chosen below the visible far-wall height of the smallest target pit
  /// (min(d, w*tan(gamma)) ~= 0.11 m at 2.5 m), with margin.
  double min_depth = 0.05;
  /// Minimum height above the ground plane for an elevated obstacle [m].
  double overhang_min_height = 0.05;
  /// Maximum height above the ground plane for an elevated obstacle [m]
  /// (the robot height; anything above cannot collide).
  double overhang_max_height = 1.20;
  /// Euclidean cluster tolerance [m].
  double cluster_tolerance = 0.05;
  /// Minimum number of points for a cluster to be reported.
  int min_cluster_size = 8;
  /// Upper bound on cluster size (safety).
  int max_cluster_size = 100000;
};

/// Classifies non-ground points into negative obstacles and elevated obstacles.
///
/// Two cues are used, both relative to the ground plane via signed distance:
///   - points below the plane by more than `min_depth` are negative obstacles
///     (the visible far wall / floor of a pit);
///   - points above the plane within the robot height band are elevated
///     obstacles (table tops, boxes, walls).
/// Each class is clustered with Euclidean clustering so that a reported
/// obstacle is a spatially connected region rather than isolated points.
class NegativeObstacleExtractor
{
public:
  explicit NegativeObstacleExtractor(const NegativeObstacleExtractorParams & params);
  ~NegativeObstacleExtractor() = default;

  NegativeObstacleExtractor(const NegativeObstacleExtractor &) = delete;
  NegativeObstacleExtractor & operator=(const NegativeObstacleExtractor &) = delete;

  /// Extracts `negative` and `elevated` obstacle clouds from `non_ground`.
  void extract(
    const pcl::PointCloud<PointT>::ConstPtr & non_ground,
    const GroundPlane & plane,
    pcl::PointCloud<PointT> & negative,
    pcl::PointCloud<PointT> & elevated);

private:
  /// Clusters `input` and keeps only clusters of at least `min_cluster_size`.
  void cluster(
    const pcl::PointCloud<PointT>::ConstPtr & input,
    pcl::PointCloud<PointT> & output);

  NegativeObstacleExtractorParams params_;
  pcl::search::KdTree<PointT>::Ptr tree_;
};

}  // namespace negative_obstacle_detector

#endif  // NEGATIVE_OBSTACLE_DETECTOR__NEGATIVE_OBSTACLE_EXTRACTOR_HPP_
