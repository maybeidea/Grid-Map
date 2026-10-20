#include "plan_env/grid_map.hpp"

#include <cmath>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

namespace plan_env {

bool GridMap::prepareMapPoints(const PreparedCloud &prepared,
                               const Eigen::Isometry3f &pose,
                               std::vector<Eigen::Vector3f> *points,
                               Eigen::Vector3f *camera_position,
                               const bool high_resolution) const {
  if (!points || !camera_position) {
    return false;
  }
  points->clear();
  points->reserve(prepared.sensor_points.size());
  *camera_position = pose.translation();
  const TaskRegion &region = task_region_;
  for (const auto &point : prepared.sensor_points) {
    const Eigen::Vector3f point_map = pose * point;
    const bool clearing_ground =
        cloud_low_z_clear_rays_ && point_map.z() < cloud_min_map_z_;
    const bool inside =
        high_resolution
            ? region.containsOriginal(
                  point_map.x(), point_map.y(),
                  clearing_ground ? std::max(point_map.z(), region.z_min)
                                  : point_map.z())
            : region.contains(point_map.x(), point_map.y(),
                              clearing_ground
                                  ? std::max(point_map.z(), region.z_min)
                                  : point_map.z());
    if (!inside) {
      continue;
    }
    if (point_map.z() < cloud_min_map_z_ && !cloud_low_z_clear_rays_) {
      continue;
    }
    points->push_back(point_map);
  }
  return !points->empty();
}

bool GridMap::updateLowResolutionMap(const PreparedCloud &prepared) {
  std::vector<Eigen::Vector3f> points;
  Eigen::Vector3f camera_position;
  if (!prepareMapPoints(prepared, prepared.prior_pose, &points,
                        &camera_position, false)) {
    return false;
  }
  std::vector<RayObservation> observations;
  observations.reserve(points.size());
  for (const auto &point : points) {
    observations.push_back(
        RayObservation{camera_position, point, point.z() < cloud_min_map_z_});
  }
  if (!integrateRayObservations(observations, &low_occupancy_voxels_,
                                low_tsdf_enabled_ ? &low_tsdf_volume_ : nullptr,
                                low_grid_geometry_, low_occupancy_model_)) {
    return false;
  }
  if (!low_esdf_volume_.compute(low_occupancy_voxels_, low_occupancy_model_)) {
    return false;
  }
  low_map_dirty_ = true;
  return true;
}

bool GridMap::processPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
    PreparedCloud *prepared) {
  if (!prepared) {
    return false;
  }
  prepared->sensor_points.clear();
  if (!cloud || cloud->data.empty() || cloud->header.frame_id.empty()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap rejected cloud: null/empty data or missing frame_id");
    }
    return false;
  }

  // Remove invalid points before voxel filtering.
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::fromROSMsg(*cloud, input_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr finite_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  finite_cloud->points.reserve(input_cloud.points.size());
  for (const auto &point : input_cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      finite_cloud->points.push_back(point);
    }
  }
  finite_cloud->width = static_cast<uint32_t>(finite_cloud->points.size());
  finite_cloud->height = 1;
  finite_cloud->is_dense = true;
  if (finite_cloud->empty()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap rejected cloud: all %u input points are non-finite",
          static_cast<unsigned int>(input_cloud.points.size()));
    }
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(finite_cloud);
  voxel_filter.setLeafSize(cloud_leaf_size_, cloud_leaf_size_,
                           cloud_leaf_size_);
  voxel_filter.filter(filtered_cloud);
  if (filtered_cloud.empty()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap rejected cloud: voxel filter removed all %u finite points "
          "(leaf=%.4fm)",
          static_cast<unsigned int>(finite_cloud->points.size()),
          cloud_leaf_size_);
    }
    return false;
  }

  const std::string source_frame = cloud->header.frame_id.front() == '/'
                                       ? cloud->header.frame_id.substr(1)
                                       : cloud->header.frame_id;
  if (source_frame.empty()) {
    return false;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &error) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap rejected cloud: TF lookup %s <- %s failed: %s",
          map_frame_.c_str(), source_frame.c_str(), error.what());
    }
    return false;
  }

  const auto &translation = transform.transform.translation;
  const auto &rotation = transform.transform.rotation;
  const Eigen::Quaternionf map_from_camera(
      static_cast<float>(rotation.w), static_cast<float>(rotation.x),
      static_cast<float>(rotation.y), static_cast<float>(rotation.z));
  const Eigen::Matrix3f rotation_matrix =
      map_from_camera.normalized().toRotationMatrix();
  const Eigen::Vector3f translation_vector(static_cast<float>(translation.x),
                                           static_cast<float>(translation.y),
                                           static_cast<float>(translation.z));

  std::vector<Eigen::Vector3f> sensor_points;
  sensor_points.reserve(filtered_cloud.points.size());
  for (const auto &point : filtered_cloud.points) {
    const Eigen::Vector3f point_camera(point.x, point.y, point.z);
    if (std::abs(point_camera.x()) >= local_update_range_.x() ||
        std::abs(point_camera.y()) >= local_update_range_.y() ||
        std::abs(point_camera.z()) >= local_update_range_.z() ||
        point_camera.squaredNorm() <= cloud_min_range_ * cloud_min_range_) {
      continue;
    }
    sensor_points.push_back(point_camera);
  }
  if (sensor_points.empty()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap rejected cloud: no points remain after sensor range filter "
          "(input=%u, min_range=%.3fm, range=[%.2f, %.2f, %.2f]m)",
          static_cast<unsigned int>(filtered_cloud.points.size()),
          cloud_min_range_, local_update_range_.x(), local_update_range_.y(),
          local_update_range_.z());
    }
    return false;
  }

  prepared->sensor_points = std::move(sensor_points);
  prepared->prior_pose = Eigen::Isometry3f::Identity();
  prepared->prior_pose.linear() = rotation_matrix;
  prepared->prior_pose.translation() = translation_vector;

  // The chassis/base pose is the preferred motion signal. A camera-mounted
  // pose is a safe fallback for setups that do not publish motion_frame_.
  prepared->motion_position = translation_vector;
  prepared->motion_orientation = map_from_camera;
  if (!motion_frame_.empty() && motion_frame_ != source_frame) {
    try {
      const auto motion_transform = tf_buffer_->lookupTransform(
          map_frame_, motion_frame_, cloud->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
      const auto &motion_translation = motion_transform.transform.translation;
      const auto &motion_rotation = motion_transform.transform.rotation;
      prepared->motion_position =
          Eigen::Vector3f(static_cast<float>(motion_translation.x),
                          static_cast<float>(motion_translation.y),
                          static_cast<float>(motion_translation.z));
      prepared->motion_orientation =
          Eigen::Quaternionf(static_cast<float>(motion_rotation.w),
                             static_cast<float>(motion_rotation.x),
                             static_cast<float>(motion_rotation.y),
                             static_cast<float>(motion_rotation.z))
              .normalized();
    } catch (const tf2::TransformException &error) {
      // Keep the cloud-frame pose fallback. It preserves operation for
      // systems that only publish a camera TF tree.
      if (node_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "GridMap motion TF lookup %s <- %s failed; using cloud pose "
            "fallback: %s",
            map_frame_.c_str(), motion_frame_.c_str(), error.what());
      }
    }
  }
  return true;
}

}  // namespace plan_env
