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

#include "negative_obstacle_detector/ground_segmenter.hpp"

namespace
{

using negative_obstacle_detector::GroundSegmenter;
using negative_obstacle_detector::GroundSegmenterParams;
using negative_obstacle_detector::PointT;

constexpr float kGroundZ = -0.15F;

}  // namespace

/// A synthetic horizontal plane with a small block above it is split into
/// ground (the plane) and non_ground (the block).
TEST(GroundSegmenterTest, SegmentsHorizontalPlane)
{
  auto input = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

  // Ground plane: a 41x41 grid spanning [-2, 2] at z = -0.15.
  constexpr int kGrid = 41;
  for (int i = 0; i < kGrid; ++i) {
    for (int j = 0; j < kGrid; ++j) {
      PointT point;
      point.x = -2.0F + static_cast<float>(i) * 0.1F;
      point.y = -2.0F + static_cast<float>(j) * 0.1F;
      point.z = kGroundZ;
      input->push_back(point);
    }
  }

  // A small block sitting 0.15 m above the plane.
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      PointT point;
      point.x = -0.04F + static_cast<float>(i) * 0.02F;
      point.y = -0.04F + static_cast<float>(j) * 0.02F;
      point.z = 0.0F;
      input->push_back(point);
    }
  }

  GroundSegmenterParams params;
  GroundSegmenter segmenter(params);

  pcl::PointCloud<PointT> ground;
  pcl::PointCloud<PointT> non_ground;
  ASSERT_TRUE(segmenter.segment(input, ground, non_ground));

  const auto & plane = segmenter.plane();
  EXPECT_TRUE(plane.valid);
  EXPECT_NEAR(plane.a, 0.0F, 0.05F);
  EXPECT_NEAR(plane.b, 0.0F, 0.05F);
  EXPECT_NEAR(plane.c, 1.0F, 0.05F);
  EXPECT_NEAR(plane.d, -kGroundZ, 0.05F);

  // The ground cloud holds the plane points, the block goes to non_ground.
  EXPECT_GT(ground.size(), 1500u);
  EXPECT_GE(non_ground.size(), 20u);
  EXPECT_LT(non_ground.size(), 100u);

  for (const auto & point : ground.points) {
    EXPECT_NEAR(point.z, kGroundZ, 0.03F);
  }
  for (const auto & point : non_ground.points) {
    EXPECT_NEAR(point.z, 0.0F, 0.03F);
  }
}
