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

#include <cmath>
#include <vector>

#include "negative_obstacle_detector/free_space_detector.hpp"

namespace
{

using negative_obstacle_detector::FreeSpaceDetector;
using negative_obstacle_detector::FreeSpaceDetectorParams;
using negative_obstacle_detector::GroundPlane;
using negative_obstacle_detector::PointT;

constexpr float kGroundZ = -0.15F;

GroundPlane make_plane()
{
  GroundPlane plane;
  plane.a = 0.0F;
  plane.b = 0.0F;
  plane.c = 1.0F;
  plane.d = -kGroundZ;  // z = -0.15  =>  1*z + 0.15 = 0
  plane.valid = true;
  return plane;
}

FreeSpaceDetectorParams make_params()
{
  FreeSpaceDetectorParams params;
  params.sensor_x = 0.0;
  params.sensor_y = 0.0;
  params.sensor_z = 0.3;
  params.min_range = 0.4;
  params.max_range = 6.0;
  params.gap_threshold = 0.3;
  params.angle_bin_size = 1.0;
  return params;
}

/// Add ground points along the +x ray (angle 0) at the given ranges.
void add_ray_points(pcl::PointCloud<PointT> & cloud, const std::vector<double> & ranges)
{
  for (const double r : ranges) {
    PointT point;
    point.x = static_cast<float>(r);
    point.y = 0.0F;
    point.z = kGroundZ;
    cloud.push_back(point);
  }
}

}  // namespace

/// A radial gap in ground coverage produces a leading-edge point at the gap.
TEST(FreeSpaceDetectorTest, EmitsLeadingEdgeAtGap)
{
  FreeSpaceDetector detector(make_params());

  auto ground = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  // Near segment 1.0..1.5 m and far segment 2.0..2.5 m, leaving a 0.5 m gap.
  std::vector<double> near_ranges = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
  std::vector<double> far_ranges = {2.0, 2.1, 2.2, 2.3, 2.4, 2.5};
  add_ray_points(*ground, near_ranges);
  add_ray_points(*ground, far_ranges);

  pcl::PointCloud<PointT> edges;
  detector.detect(ground, make_plane(), edges);

  ASSERT_GE(edges.size(), 1u);

  // The leading edge is the smaller range of the gap: (1.5, 0, -0.15).
  bool found = false;
  for (const auto & edge : edges.points) {
    const double dx = static_cast<double>(edge.x) - 1.5;
    const double dy = static_cast<double>(edge.y) - 0.0;
    if (std::hypot(dx, dy) < 0.1) {
      found = true;
      EXPECT_NEAR(edge.z, kGroundZ, 1e-3F);
    }
  }
  EXPECT_TRUE(found);
}

/// Continuous ground coverage produces no leading-edge points.
TEST(FreeSpaceDetectorTest, NoGapNoEdges)
{
  FreeSpaceDetector detector(make_params());

  auto ground = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  std::vector<double> ranges;
  for (double r = 1.0; r <= 3.0; r += 0.1) {
    ranges.push_back(r);
  }
  add_ray_points(*ground, ranges);

  pcl::PointCloud<PointT> edges;
  detector.detect(ground, make_plane(), edges);

  EXPECT_EQ(edges.size(), 0u);
}
