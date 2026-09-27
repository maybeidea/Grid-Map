#include "grid_map.hpp"

#include <Eigen/Eigen>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#if __has_include(<tf2/exceptions.h>)
#include <tf2/exceptions.h>
#else
#include <tf2/exceptions.hpp>
#endif

namespace {

constexpr double kDefaultDistance = 10000.0;

bool shouldClearOnRayStaleOccupied(const int hit_count, const int miss_count,
                                   const int min_miss) {
  return hit_count == 0 && miss_count >= min_miss;
}

bool pointInPolygonXy(double x, double y,
                      const std::vector<Eigen::Vector2d> &poly) {
  const int n = static_cast<int>(poly.size());
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double yi = poly[static_cast<size_t>(i)].y();
    const double yj = poly[static_cast<size_t>(j)].y();
    const bool intersect = ((yi > y) != (yj > y));
    if (!intersect) {
      continue;
    }
    const double xi = poly[static_cast<size_t>(i)].x();
    const double xj = poly[static_cast<size_t>(j)].x();
    const double den = yj - yi;
    if (x < (xj - xi) * (y - yi) / den + xi) {
      inside = !inside;
    }
  }
  return inside;
}

} // namespace

GridMap::GridMap()
: latest_esdf_snapshot_(std::make_shared<EsdfSnapshot>()),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
  mapping_thread_ = std::thread(&GridMap::mappingLoop, this);
}

GridMap::~GridMap() {
  {
    std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
    stopping_ = true;
    pending_cloud_.reset();
  }
  pending_cloud_cv_.notify_one();
  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
}

bool GridMap::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  if (!cloud) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
    if (stopping_) {
      return false;
    }
    pending_cloud_ = std::move(cloud);
  }
  pending_cloud_cv_.notify_one();
  return true;
}

std::shared_ptr<const EsdfSnapshot> GridMap::snapshot() const {
  return std::atomic_load(&latest_esdf_snapshot_);
}

void GridMap::mappingLoop() {
  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::unique_lock<std::mutex> lock(pending_cloud_mutex_);
      pending_cloud_cv_.wait(lock,
                             [this] { return stopping_ || pending_cloud_; });
      if (stopping_) {
        return;
      }
      cloud = std::move(pending_cloud_);
      pending_cloud_.reset();
    }

    processPointCloud(cloud);
    publishEsdfSnapshot();
  }
}

void GridMap::processPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  if (!cloud || cloud->data.empty() || cloud->header.frame_id.empty()) {
    return;
  }

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
    return;
  }

  constexpr float kCloudLeafSize = 0.1f;
  constexpr float kCloudMinRange = 0.2f;
  const Eigen::Vector3f local_update_range(5.0f, 5.0f, 3.0f);
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(finite_cloud);
  voxel_filter.setLeafSize(kCloudLeafSize, kCloudLeafSize, kCloudLeafSize);
  voxel_filter.filter(filtered_cloud);
  if (filtered_cloud.empty()) {
    return;
  }

  const std::string source_frame = cloud->header.frame_id.front() == '/'
                                       ? cloud->header.frame_id.substr(1)
                                       : cloud->header.frame_id;
  if (source_frame.empty()) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &) {
    return;
  }

  const auto &translation = transform.transform.translation;
  const auto &rotation = transform.transform.rotation;
  const Eigen::Quaternionf map_from_camera(
      static_cast<float>(rotation.w), static_cast<float>(rotation.x),
      static_cast<float>(rotation.y), static_cast<float>(rotation.z));
  const Eigen::Matrix3f rotation_matrix = map_from_camera.normalized().toRotationMatrix();
  const Eigen::Vector3f translation_vector(
      static_cast<float>(translation.x), static_cast<float>(translation.y),
      static_cast<float>(translation.z));

  std::vector<Eigen::Vector3f> transformed_points;
  transformed_points.reserve(filtered_cloud.points.size());
  for (const auto &point : filtered_cloud.points) {
    const Eigen::Vector3f point_camera(point.x, point.y, point.z);
    if (std::abs(point_camera.x()) >= local_update_range.x() ||
        std::abs(point_camera.y()) >= local_update_range.y() ||
        std::abs(point_camera.z()) >= local_update_range.z() ||
        point_camera.norm() <= kCloudMinRange) {
      continue;
    }
    transformed_points.push_back(rotation_matrix * point_camera + translation_vector);
  }
  cloud_points_ = std::move(transformed_points);
  camera_position_ = translation_vector;
}

void GridMap::publishEsdfSnapshot() {
  auto next = std::make_shared<EsdfSnapshot>();
  next->geometry = grid_geometry_;
  next->esdf_voxels = esdf_voxels_;
  next->version = ++snapshot_version_;
  std::atomic_store(&latest_esdf_snapshot_,
                    std::shared_ptr<const EsdfSnapshot>(std::move(next)));
}

int main() {
  return 0;
}