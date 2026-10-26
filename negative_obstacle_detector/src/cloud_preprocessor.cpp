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

#include "negative_obstacle_detector/cloud_preprocessor.hpp"

#include <cmath>

namespace negative_obstacle_detector
{

CloudPreprocessor::CloudPreprocessor(const PreprocessorParams & params)
: params_(params),
  finite_(new pcl::PointCloud<PointT>()),
  cropped_(new pcl::PointCloud<PointT>()),
  downsampled_(new pcl::PointCloud<PointT>())
{
  pass_x_.setFilterFieldName("x");
  pass_x_.setFilterLimits(params_.min_range, params_.max_range);

  pass_y_.setFilterFieldName("y");
  pass_y_.setFilterLimits(-params_.half_width, params_.half_width);

  pass_z_.setFilterFieldName("z");
  pass_z_.setFilterLimits(params_.min_height, params_.max_height);

  voxel_.setLeafSize(
    params_.voxel_leaf_size, params_.voxel_leaf_size, params_.voxel_leaf_size);
}

void CloudPreprocessor::process(
  const pcl::PointCloud<PointT>::ConstPtr & input,
  pcl::PointCloud<PointT> & output)
{
  // Stage 1: finite filter. `pcl::removeNaNFromPointCloud` only removes NaNs;
  // sensors also emit +inf on no-return rays, so filter all three coordinates
  // explicitly.
  finite_->clear();
  finite_->reserve(input->size());
  for (const auto & point : input->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite_->push_back(point);
    }
  }

  // Stage 2: PassThrough cropping on x, y and z.
  pass_x_.setInputCloud(finite_);
  pass_x_.filter(*cropped_);
  pass_y_.setInputCloud(cropped_);
  pass_y_.filter(*cropped_);
  pass_z_.setInputCloud(cropped_);
  pass_z_.filter(*cropped_);

  // Stage 3: VoxelGrid downsampling.
  voxel_.setInputCloud(cropped_);
  voxel_.filter(*downsampled_);

  // Copy the result into the caller-provided buffer.
  output = *downsampled_;
}

}  // namespace negative_obstacle_detector
