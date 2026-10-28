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
#include <limits>

#include "negative_obstacle_detector/cloud_preprocessor.hpp"

namespace
{

using negative_obstacle_detector::CloudPreprocessor;
using negative_obstacle_detector::PointT;
using negative_obstacle_detector::PreprocessorParams;

pcl::PointCloud<PointT>::Ptr make_cloud()
{
  return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
}

}  // namespace

/// The finite filter drops NaN and +/-inf, keeping only finite points.
TEST(CloudPreprocessorTest, FiniteFilterDropsNonFinite)
{
  PreprocessorParams params;
  CloudPreprocessor preprocessor(params);

  auto input = make_cloud();
  // Valid points inside the default ROI.
  input->push_back(PointT{1.0F, 0.0F, 0.0F});
  input->push_back(PointT{2.0F, 0.5F, -0.2F});
  input->push_back(PointT{3.0F, -0.5F, 0.2F});
  // NaN and +/-inf points that must be dropped.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  input->push_back(PointT{nan, 0.0F, 0.0F});
  input->push_back(PointT{1.0F, nan, 0.0F});
  input->push_back(PointT{1.0F, 0.0F, nan});
  input->push_back(PointT{inf, 0.0F, 0.0F});
  input->push_back(PointT{1.0F, -inf, 0.0F});

  pcl::PointCloud<PointT> output;
  preprocessor.process(input, output);

  EXPECT_EQ(output.size(), 3u);
  for (const auto & point : output.points) {
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
    EXPECT_TRUE(std::isfinite(point.z));
  }
}

/// PassThrough crops the cloud to the x/y/z region of interest.
TEST(CloudPreprocessorTest, PassThroughCropsToRoi)
{
  PreprocessorParams params;
  params.min_range = 0.5;
  params.max_range = 2.0;
  params.half_width = 1.0;
  params.min_height = -0.5;
  params.max_height = 0.5;
  params.voxel_leaf_size = 0.01;
  CloudPreprocessor preprocessor(params);

  auto input = make_cloud();
  input->push_back(PointT{1.0F, 0.0F, 0.0F});   // inside, kept
  input->push_back(PointT{0.2F, 0.0F, 0.0F});   // x < min_range
  input->push_back(PointT{3.0F, 0.0F, 0.0F});   // x > max_range
  input->push_back(PointT{1.0F, 1.5F, 0.0F});   // y > half_width
  input->push_back(PointT{1.0F, -1.5F, 0.0F});  // y < -half_width
  input->push_back(PointT{1.0F, 0.0F, 0.8F});   // z > max_height
  input->push_back(PointT{1.0F, 0.0F, -0.8F});  // z < min_height

  pcl::PointCloud<PointT> output;
  preprocessor.process(input, output);

  ASSERT_EQ(output.size(), 1u);
  EXPECT_FLOAT_EQ(output.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(output.points[0].y, 0.0F);
  EXPECT_FLOAT_EQ(output.points[0].z, 0.0F);
}

/// VoxelGrid downsamples a dense cloud by merging points into voxel centroids.
TEST(CloudPreprocessorTest, VoxelGridReducesPointCount)
{
  PreprocessorParams params;
  params.voxel_leaf_size = 0.05;
  CloudPreprocessor preprocessor(params);

  auto input = make_cloud();
  // A dense 21x21x21 lattice inside a 2 cm cube, all well within a single
  // voxel centered near (1, 0, 0).
  for (int i = 0; i < 21; ++i) {
    for (int j = 0; j < 21; ++j) {
      for (int k = 0; k < 21; ++k) {
        PointT point;
        point.x = 1.0F + static_cast<float>(i - 10) * 0.001F;
        point.y = static_cast<float>(j - 10) * 0.001F;
        point.z = static_cast<float>(k - 10) * 0.001F;
        input->push_back(point);
      }
    }
  }
  const std::size_t input_size = input->size();

  pcl::PointCloud<PointT> output;
  preprocessor.process(input, output);

  EXPECT_LT(output.size(), input_size);
  // The 21x21x21 lattice collapses to a handful of voxel centroids.
  EXPECT_LT(output.size(), input_size / 100);
}
