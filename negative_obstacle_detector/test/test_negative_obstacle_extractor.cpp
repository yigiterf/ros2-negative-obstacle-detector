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

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "negative_obstacle_detector/negative_obstacle_extractor.hpp"

namespace
{

using negative_obstacle_detector::GroundPlane;
using negative_obstacle_detector::NegativeObstacleExtractor;
using negative_obstacle_detector::NegativeObstacleExtractorParams;
using negative_obstacle_detector::PointT;

// Plane z = -0.15 (a=0, b=0, c=1, d=0.15): signed_distance = z + 0.15.
GroundPlane make_plane()
{
  GroundPlane plane;
  plane.a = 0.0F;
  plane.b = 0.0F;
  plane.c = 1.0F;
  plane.d = 0.15F;
  plane.valid = true;
  return plane;
}

/// A connected cluster of `count` points starting at `z`.
void add_cluster(pcl::PointCloud<PointT> & cloud, int count, float z, float x0)
{
  for (int i = 0; i < count; ++i) {
    PointT point;
    point.x = x0 + static_cast<float>(i) * 0.01F;
    point.y = 0.0F;
    point.z = z;
    cloud.push_back(point);
  }
}

}  // namespace

/// Below-plane clusters become negative obstacles, in-band clusters become
/// elevated obstacles, and clusters smaller than min_cluster_size are dropped.
TEST(NegativeObstacleExtractorTest, ClassifiesBySignedDistanceAndClusterSize)
{
  NegativeObstacleExtractorParams params;
  NegativeObstacleExtractor extractor(params);

  auto non_ground = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

  // Negative obstacle: 8 connected points well below the plane (z = -0.5).
  add_cluster(*non_ground, 8, -0.5F, 0.0F);
  // Elevated obstacle: 8 connected points above the plane (z = 0.0).
  add_cluster(*non_ground, 8, 0.0F, 1.0F);
  // Isolated below-plane points, each its own cluster below min_cluster_size.
  add_cluster(*non_ground, 1, -0.5F, 5.0F);
  add_cluster(*non_ground, 1, -0.5F, 6.0F);
  add_cluster(*non_ground, 1, -0.5F, 7.0F);

  pcl::PointCloud<PointT> negative;
  pcl::PointCloud<PointT> elevated;
  extractor.extract(non_ground, make_plane(), negative, elevated);

  EXPECT_EQ(negative.size(), 8u);
  EXPECT_EQ(elevated.size(), 8u);

  for (const auto & point : negative.points) {
    EXPECT_LT(point.z, -0.2F);
  }
  for (const auto & point : elevated.points) {
    EXPECT_NEAR(point.z, 0.0F, 1e-3F);
  }
}
