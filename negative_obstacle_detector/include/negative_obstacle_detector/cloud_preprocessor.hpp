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

#ifndef NEGATIVE_OBSTACLE_DETECTOR__CLOUD_PREPROCESSOR_HPP_
#define NEGATIVE_OBSTACLE_DETECTOR__CLOUD_PREPROCESSOR_HPP_

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace negative_obstacle_detector
{

/// Geometry-only point type used throughout the pipeline.
///
/// Negative-obstacle detection is a purely geometric problem, so colour and
/// intensity are discarded on entry. Using a single point type keeps the
/// pipeline sensor-agnostic (RGB-D and 3D lidar both produce `x, y, z`).
using PointT = pcl::PointXYZ;

/// Tunable parameters for the pre-processing stage.
struct PreprocessorParams
{
  /// Near clipping distance in front of the sensor [m].
  double min_range = 0.4;
  /// Far clipping distance in front of the sensor [m].
  double max_range = 6.0;
  /// Lower vertical bound, relative to the sensor frame [m]. Chosen below the
  /// deepest detectable pit floor so pit geometry is not cropped away.
  double min_height = -0.6;
  /// Upper vertical bound, relative to the sensor frame [m]. Equal to the robot
  /// height; points above this cannot collide with the robot.
  double max_height = 1.2;
  /// Half-width of the lateral region of interest [m].
  double half_width = 3.0;
  /// VoxelGrid leaf size [m].
  double voxel_leaf_size = 0.02;
};

/// Crops and downsamples an input cloud.
///
/// The pipeline runs in three stages:
///   1. finite filter     - drops NaN and +inf (emitted by sensors on no-return
///                          rays) so downstream PCL algorithms stay well-defined;
///   2. PassThrough       - crops to the region of interest on x, y and z;
///   3. VoxelGrid         - downsamples by leaf-sized voxel centroids.
///
/// Filter objects are constructed once and reused across calls, and all
/// working buffers are owned by the instance, so steady-state processing does
/// not allocate on the heap per frame.
class CloudPreprocessor
{
public:
  explicit CloudPreprocessor(const PreprocessorParams & params);
  ~CloudPreprocessor() = default;

  CloudPreprocessor(const CloudPreprocessor &) = delete;
  CloudPreprocessor & operator=(const CloudPreprocessor &) = delete;

  /// Preprocesses `input` and writes the result into `output`.
  ///
  /// `output` is cleared and resized as needed; the caller may pass the same
  /// cloud instance across frames.
  void process(
    const pcl::PointCloud<PointT>::ConstPtr & input,
    pcl::PointCloud<PointT> & output);

private:
  PreprocessorParams params_;

  // Reused working buffers.
  pcl::PointCloud<PointT>::Ptr finite_;
  pcl::PointCloud<PointT>::Ptr cropped_;
  pcl::PointCloud<PointT>::Ptr downsampled_;

  // Reused filter objects.
  pcl::PassThrough<PointT> pass_x_;
  pcl::PassThrough<PointT> pass_y_;
  pcl::PassThrough<PointT> pass_z_;
  pcl::VoxelGrid<PointT> voxel_;
};

}  // namespace negative_obstacle_detector

#endif  // NEGATIVE_OBSTACLE_DETECTOR__CLOUD_PREPROCESSOR_HPP_
