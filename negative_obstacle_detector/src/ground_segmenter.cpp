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

#include "negative_obstacle_detector/ground_segmenter.hpp"

#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <vector>

namespace negative_obstacle_detector
{

GroundSegmenter::GroundSegmenter(const GroundSegmenterParams & params)
: params_(params),
  inliers_(new pcl::PointIndices()),
  coefficients_(new pcl::ModelCoefficients())
{
  const Eigen::Vector3f gravity_axis(0.0F, 0.0F, 1.0F);

  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setAxis(gravity_axis);
  seg_.setEpsAngle(params_.angle_tolerance);
  seg_.setDistanceThreshold(params_.distance_threshold);
  seg_.setMaxIterations(params_.max_iterations);
}

bool GroundSegmenter::segment(
  const pcl::PointCloud<PointT>::ConstPtr & input,
  pcl::PointCloud<PointT> & ground,
  pcl::PointCloud<PointT> & non_ground)
{
  inliers_->indices.clear();
  coefficients_->values.clear();

  seg_.setInputCloud(input);
  seg_.segment(*inliers_, *coefficients_);

  if (inliers_->indices.empty() || coefficients_->values.size() != 4) {
    plane_ = GroundPlane{};
    return false;
  }

  // Normalize (a, b, c) to a unit vector so signed_distance() is exact.
  const float a = coefficients_->values[0];
  const float b = coefficients_->values[1];
  const float c = coefficients_->values[2];
  const float d = coefficients_->values[3];
  const float norm = std::sqrt(a * a + b * b + c * c);
  if (norm < 1e-6F) {
    plane_ = GroundPlane{};
    return false;
  }

  plane_.a = a / norm;
  plane_.b = b / norm;
  plane_.c = c / norm;
  plane_.d = d / norm;
  plane_.valid = true;

  // RANSAC's eigen decomposition returns an arbitrary normal sign. Orient the
  // normal to point up (+z) so that signed_distance() is negative below the
  // plane, which the negative-obstacle extractor and the free-space detector
  // both rely on.
  if (plane_.c < 0.0F) {
    plane_.a = -plane_.a;
    plane_.b = -plane_.b;
    plane_.c = -plane_.c;
    plane_.d = -plane_.d;
  }

  // A "ground" plane must be dominated by a vertical normal; reject degenerate
  // near-vertical fits that RANSAC occasionally returns.
  if (std::abs(plane_.c) < 0.8F) {
    plane_ = GroundPlane{};
    return false;
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers_);
  extract.setNegative(false);
  extract.filter(ground);
  extract.setNegative(true);
  extract.filter(non_ground);

  return true;
}

}  // namespace negative_obstacle_detector
