#include "plan_env/registration.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace registration {
namespace {

bool validConfig(const Config &config) {
  return std::isfinite(config.max_correspondence_distance) &&
         config.max_correspondence_distance > 0.0f &&
         config.maximum_iterations > 0 &&
         std::isfinite(config.transformation_epsilon) &&
         config.transformation_epsilon >= 0.0f &&
         std::isfinite(config.euclidean_fitness_epsilon) &&
         config.euclidean_fitness_epsilon >= 0.0f &&
         std::isfinite(config.max_fitness) && config.max_fitness >= 0.0f &&
         std::isfinite(config.min_overlap) && config.min_overlap >= 0.0f &&
         config.min_overlap <= 1.0f &&
         std::isfinite(config.max_translation_correction) &&
         config.max_translation_correction >= 0.0f &&
         std::isfinite(config.max_rotation_correction_rad) &&
         config.max_rotation_correction_rad >= 0.0f &&
         config.min_target_points > 0;
}

} // namespace

PointCloudRegistrationBackend::PointCloudRegistrationBackend(
    const Config &config) {
  if (!configure(config)) {
    config_ = Config{};
  }
}

bool PointCloudRegistrationBackend::configure(const Config &config) {
  if (!validConfig(config)) {
    return false;
  }
  config_ = config;
  return true;
}

const Config &PointCloudRegistrationBackend::config() const { return config_; }

Result PointCloudRegistrationBackend::align(
    const std::vector<Eigen::Vector3f> &source,
    const std::vector<Eigen::Vector3f> &target,
    const Eigen::Isometry3f &initial_pose) const {
  Result result;
  result.pose = initial_pose;
  if (!config_.enabled || source.empty() ||
      target.size() < config_.min_target_points ||
      !initial_pose.matrix().allFinite()) {
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  source_cloud->points.reserve(source.size());
  target_cloud->points.reserve(target.size());
  for (const auto &point : source) {
    if (point.allFinite()) {
      source_cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
  }
  for (const auto &point : target) {
    if (point.allFinite()) {
      target_cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
  }
  if (source_cloud->empty() ||
      target_cloud->size() < config_.min_target_points) {
    return result;
  }
  source_cloud->width = static_cast<std::uint32_t>(source_cloud->size());
  source_cloud->height = 1;
  target_cloud->width = static_cast<std::uint32_t>(target_cloud->size());
  target_cloud->height = 1;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
  icp.setMaximumIterations(config_.maximum_iterations);
  icp.setTransformationEpsilon(config_.transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned, initial_pose.matrix());

  result.converged = icp.hasConverged();
  result.fitness = static_cast<float>(icp.getFitnessScore());
  const Eigen::Matrix4f matrix = icp.getFinalTransformation();
  if (!result.converged || !matrix.allFinite()) {
    return result;
  }
  result.pose.matrix() = matrix;

  const Eigen::Isometry3f correction = result.pose * initial_pose.inverse();
  result.translation_correction = correction.translation().norm();
  result.rotation_correction_rad =
      Eigen::AngleAxisf(correction.rotation()).angle();

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target_cloud);
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  std::size_t matches = 0;
  for (const auto &point : aligned.points) {
    if (tree.nearestKSearch(point, 1, indices, distances) > 0 &&
        distances[0] <= config_.max_correspondence_distance *
                             config_.max_correspondence_distance) {
      ++matches;
    }
  }
  result.overlap = aligned.empty()
                       ? 0.0f
                       : static_cast<float>(matches) /
                             static_cast<float>(aligned.size());
  result.accepted =
      std::isfinite(result.fitness) && result.fitness <= config_.max_fitness &&
      result.overlap >= config_.min_overlap &&
      result.translation_correction <= config_.max_translation_correction &&
      result.rotation_correction_rad <= config_.max_rotation_correction_rad;
  return result;
}

} // namespace registration
