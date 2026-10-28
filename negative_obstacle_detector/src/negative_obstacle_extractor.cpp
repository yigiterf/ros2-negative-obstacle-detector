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

#include "negative_obstacle_detector/negative_obstacle_extractor.hpp"

#include <pcl/segmentation/extract_clusters.h>

#include <vector>

namespace negative_obstacle_detector
{

NegativeObstacleExtractor::NegativeObstacleExtractor(
  const NegativeObstacleExtractorParams & params)
: params_(params),
  tree_(new pcl::search::KdTree<PointT>())
{
}

void NegativeObstacleExtractor::extract(
  const pcl::PointCloud<PointT>::ConstPtr & non_ground,
  const GroundPlane & plane,
  pcl::PointCloud<PointT> & negative,
  pcl::PointCloud<PointT> & elevated)
{
  // Split by signed distance relative to the ground plane.
  auto below = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  auto above = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  below->reserve(non_ground->size());
  above->reserve(non_ground->size());

  for (const auto & point : non_ground->points) {
    const float dist = plane.signed_distance(point);
    const bool is_elevated =
      dist > params_.overhang_min_height &&
      dist < params_.overhang_max_height;
    if (dist < -params_.min_depth) {
      below->push_back(point);
    } else if (is_elevated) {
      above->push_back(point);
    }
  }

  cluster(below, negative);
  cluster(above, elevated);
}

void NegativeObstacleExtractor::cluster(
  const pcl::PointCloud<PointT>::ConstPtr & input,
  pcl::PointCloud<PointT> & output)
{
  output.clear();

  if (input->empty()) {
    return;
  }

  tree_->setInputCloud(input);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(params_.cluster_tolerance);
  ec.setMinClusterSize(params_.min_cluster_size);
  ec.setMaxClusterSize(params_.max_cluster_size);
  ec.setSearchMethod(tree_);
  ec.setInputCloud(input);
  ec.extract(clusters);

  output.reserve(input->size());
  for (const auto & cluster : clusters) {
    for (const int index : cluster.indices) {
      output.push_back(input->points[index]);
    }
  }
}

}  // namespace negative_obstacle_detector
