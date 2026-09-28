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

std::vector<Eigen::Vector2d> cleanPolygon(
    const std::vector<Eigen::Vector2d> &polygon) {
  std::vector<Eigen::Vector2d> result;
  result.reserve(polygon.size());
  for (const auto &point : polygon) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      continue;
    }
    if (!result.empty() && (point - result.back()).squaredNorm() < 1e-12) {
      continue;
    }
    result.push_back(point);
  }
  if (result.size() > 1 && (result.front() - result.back()).squaredNorm() < 1e-12) {
    result.pop_back();
  }
  return result;
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

bool GridMap::configure(const float resolution, const float map_size_z,
                        const float expansion_margin,
                        const float expansion_ratio) {
  if (!std::isfinite(resolution) || resolution <= 0.0f ||
      !std::isfinite(map_size_z) || map_size_z <= 0.0f ||
      !std::isfinite(expansion_margin) || expansion_margin < 0.0f ||
      !std::isfinite(expansion_ratio) || expansion_ratio < 0.0f) {
    return false;
  }
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  resolution_ = resolution;
  map_size_z_ = map_size_z;
  expansion_margin_ = expansion_margin;
  expansion_ratio_ = expansion_ratio;
  return true;
}

bool GridMap::startTaskMapping(const std::vector<Eigen::Vector2d> &polygon_xy,
                               std::string *message_out) {
  const auto polygon = cleanPolygon(polygon_xy);
  if (polygon.size() < 3) {
    if (message_out) {
      *message_out = "task polygon needs at least 3 finite unique vertices";
    }
    return false;
  }
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto &point : polygon) {
    center_x += point.x();
    center_y += point.y();
  }
  center_x /= static_cast<double>(polygon.size());
  center_y /= static_cast<double>(polygon.size());
  return startTaskMappingWithCenter(center_x, center_y, polygon, message_out);
}

bool GridMap::startTaskMapping(const double center_x, const double center_y,
                               const std::vector<Eigen::Vector2d> &polygon_xy,
                               std::string *message_out) {
  return startTaskMappingWithCenter(center_x, center_y, cleanPolygon(polygon_xy),
                                    message_out);
}

bool GridMap::startTaskMappingWithCenter(
    const double center_x, const double center_y,
    const std::vector<Eigen::Vector2d> &polygon_xy,
    std::string *message_out) {
  if (polygon_xy.size() < 3 || !std::isfinite(center_x) ||
      !std::isfinite(center_y)) {
    if (message_out) {
      *message_out = "invalid task polygon or center";
    }
    return false;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto &point : polygon_xy) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }

  std::lock_guard<std::mutex> lock(task_region_mutex_);
  const double width = std::max(max_x - min_x, static_cast<double>(resolution_));
  const double height = std::max(max_y - min_y, static_cast<double>(resolution_));
  const double expand_x = std::max(static_cast<double>(expansion_margin_),
                                   width * static_cast<double>(expansion_ratio_));
  const double expand_y = std::max(static_cast<double>(expansion_margin_),
                                   height * static_cast<double>(expansion_ratio_));
  const double expanded_width = width + 2.0 * expand_x;
  const double expanded_height = height + 2.0 * expand_y;

  task_region_ = TaskRegion{};
  task_region_.valid = true;
  task_region_.center = Eigen::Vector2f(static_cast<float>(center_x),
                                        static_cast<float>(center_y));
  task_region_.min = Eigen::Vector2f(static_cast<float>(center_x - expanded_width * 0.5),
                                     static_cast<float>(center_y - expanded_height * 0.5));
  task_region_.max = Eigen::Vector2f(static_cast<float>(center_x + expanded_width * 0.5),
                                     static_cast<float>(center_y + expanded_height * 0.5));
  task_region_.z_min = 0.0f;
  task_region_.z_max = map_size_z_;
  task_region_.expansion_margin = expansion_margin_;
  task_region_.expansion_ratio = expansion_ratio_;
  task_region_.source_polygon = polygon_xy;
  task_region_.expanded_polygon = {
      {task_region_.min.x(), task_region_.min.y()},
      {task_region_.max.x(), task_region_.min.y()},
      {task_region_.max.x(), task_region_.max.y()},
      {task_region_.min.x(), task_region_.max.y()}};

  grid_geometry_.resolution = resolution_;
  grid_geometry_.origin = Eigen::Vector3f(task_region_.min.x(), task_region_.min.y(),
                                          task_region_.z_min);
  grid_geometry_.size_x = std::max(1, static_cast<int>(std::ceil(expanded_width / resolution_)));
  grid_geometry_.size_y = std::max(1, static_cast<int>(std::ceil(expanded_height / resolution_)));
  grid_geometry_.size_z = std::max(1, static_cast<int>(std::ceil(map_size_z_ / resolution_)));
  clearMappingBuffers();
  const size_t voxel_count = static_cast<size_t>(grid_geometry_.size_x) *
                             static_cast<size_t>(grid_geometry_.size_y) *
                             static_cast<size_t>(grid_geometry_.size_z);
  tsdf_voxels_.assign(voxel_count, TsdfVoxel{});
  occupancy_voxels_.assign(voxel_count, OccupancyVoxel{});
  esdf_voxels_.assign(voxel_count, EsdfVoxel{});
  cloud_points_.clear();

  if (message_out) {
    std::ostringstream stream;
    stream << "task ROI started: center=[" << center_x << ", " << center_y
           << "] source_vertices=" << polygon_xy.size() << " expanded=["
           << task_region_.min.x() << ", " << task_region_.min.y() << "]..["
           << task_region_.max.x() << ", " << task_region_.max.y() << "] map_voxels=["
           << grid_geometry_.size_x << ", " << grid_geometry_.size_y << ", "
           << grid_geometry_.size_z << "]";
    *message_out = stream.str();
  }
  return true;
}

void GridMap::stopTaskMapping() {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  task_region_ = TaskRegion{};
  clearMappingBuffers();
  grid_geometry_ = GridGeometry{};
}

bool GridMap::isTaskMappingActive() const {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  return task_region_.valid;
}

TaskRegion GridMap::taskRegion() const {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  return task_region_;
}

bool GridMap::pointInTaskRegion(const float x, const float y, const float z) const {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  return task_region_.contains(x, y, z);
}

void GridMap::clearMappingBuffers() {
  tsdf_voxels_.clear();
  occupancy_voxels_.clear();
  esdf_voxels_.clear();
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
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return latest_esdf_snapshot_;
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

  // 转pcl并过滤无效点
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

  const TaskRegion region = taskRegion();
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
    const Eigen::Vector3f point_map = rotation_matrix * point_camera + translation_vector;
    if (region.valid && !region.contains(point_map.x(), point_map.y(), point_map.z())) {
      continue;
    }
    transformed_points.push_back(point_map);
  }
  cloud_points_ = std::move(transformed_points);
  camera_position_ = translation_vector;
}

void GridMap::publishEsdfSnapshot() {
  auto next = std::make_shared<EsdfSnapshot>();
  next->geometry = grid_geometry_;
  next->esdf_voxels = esdf_voxels_;
  const TaskRegion region = taskRegion();
  next->task_mapping_active = region.valid;
  next->task_roi_polygon = region.expanded_polygon;
  next->version = ++snapshot_version_;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_esdf_snapshot_ = std::shared_ptr<const EsdfSnapshot>(std::move(next));
  }
}
