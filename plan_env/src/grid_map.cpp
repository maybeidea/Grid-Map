#include "plan_env/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace plan_env {

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
  if (result.size() > 1 &&
      (result.front() - result.back()).squaredNorm() < 1e-12) {
    result.pop_back();
  }
  return result;
}

}  // namespace

GridMap::GridMap()
    : tf_buffer_(std::make_shared<tf2_ros::Buffer>(
          std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      latest_esdf_snapshot_(std::make_shared<EsdfSnapshot>()) {
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
  return true;
}

bool GridMap::configureOccupancy(const occupancy::LogOddsConfig &config) {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  return occupancy_model_.configure(config);
}

bool GridMap::configureEsdf(const esdf::EsdfConfig &config) {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  // Validate without changing the active volume. The volume is configured
  // with this saved policy when the next task ROI is started.
  if (!std::isfinite(config.max_distance) || config.max_distance < 0.0f) {
    return false;
  }
  esdf_config_ = config;
  return true;
}

bool GridMap::configureTsdf(const bool enabled,
                            const float truncation_distance) {
  if (!std::isfinite(truncation_distance) || truncation_distance <= 0.0f) {
    return false;
  }
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  tsdf_enabled_ = enabled;
  tsdf_truncation_distance_ = truncation_distance;
  return true;
}

bool GridMap::configurePointCloudFilter(const float leaf_size) {
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0f) {
    return false;
  }
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  cloud_leaf_size_ = leaf_size;
  return true;
}

bool GridMap::configureRegistration(const registration::Config &config) {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  return registration_backend_.configure(config);
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

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto &p : polygon) {
    min_x = std::min(min_x, p.x());
    min_y = std::min(min_y, p.y());
    max_x = std::max(max_x, p.x());
    max_y = std::max(max_y, p.y());
  }

  const double margin = static_cast<double>(expansion_margin_);
  min_x -= margin;
  min_y -= margin;
  max_x += margin;
  max_y += margin;

  const double width = max_x - min_x;
  const double height = max_y - min_y;

  std::lock_guard<std::mutex> lock(task_region_mutex_);

  task_region_ = TaskRegion{};
  task_region_.valid = true;
  task_region_.min = {static_cast<float>(min_x), static_cast<float>(min_y)};
  task_region_.z_min = 0.0f;
  task_region_.z_max = map_size_z_;
  task_region_.expansion_margin = expansion_margin_;
  task_region_.source_polygon = polygon;

  grid_geometry_.resolution = resolution_;
  grid_geometry_.origin = {task_region_.min.x(), task_region_.min.y(),
                           task_region_.z_min};

  grid_geometry_.size_x =
      std::max(1, static_cast<int>(std::ceil(width / resolution_)));
  grid_geometry_.size_y =
      std::max(1, static_cast<int>(std::ceil(height / resolution_)));
  grid_geometry_.size_z =
      std::max(1, static_cast<int>(std::ceil(map_size_z_ / resolution_)));

  // Use the actual voxel-aligned map boundary.
  task_region_.max = {
      task_region_.min.x() + grid_geometry_.size_x * resolution_,
      task_region_.min.y() + grid_geometry_.size_y * resolution_};

  task_region_.expanded_polygon = {
      {task_region_.min.x(), task_region_.min.y()},
      {task_region_.max.x(), task_region_.min.y()},
      {task_region_.max.x(), task_region_.max.y()},
      {task_region_.min.x(), task_region_.max.y()}};

  clearMappingBuffers();
  resetMotionState();

  const size_t voxel_count = static_cast<size_t>(grid_geometry_.size_x) *
                             static_cast<size_t>(grid_geometry_.size_y) *
                             static_cast<size_t>(grid_geometry_.size_z);

  if (tsdf_enabled_) {
    if (!tsdf_volume_.configure(grid_geometry_, tsdf_truncation_distance_)) {
      return false;
    }
  } else {
    tsdf_volume_.clear();
  }
  occupancy_voxels_.assign(voxel_count, OccupancyVoxel{});
  if (!esdf_volume_.configure(grid_geometry_, esdf_config_)) {
    occupancy_voxels_.clear();
    tsdf_volume_.clear();
    return false;
  }
  cloud_points_.clear();

  if (message_out) {
    std::ostringstream stream;
    stream << "task ROI started: source_vertices=" << polygon.size()
           << " expanded=[" << task_region_.min.x() << ", "
           << task_region_.min.y() << "]..[" << task_region_.max.x() << ", "
           << task_region_.max.y() << "] map_voxels=[" << grid_geometry_.size_x
           << ", " << grid_geometry_.size_y << ", " << grid_geometry_.size_z
           << "]";
    *message_out = stream.str();
  }

  return true;
}

void GridMap::stopTaskMapping() {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  task_region_ = TaskRegion{};
  clearMappingBuffers();
  resetMotionState();
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

bool GridMap::pointInTaskRegion(const float x, const float y,
                                const float z) const {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  return task_region_.contains(x, y, z);
}

void GridMap::clearMappingBuffers() {
  tsdf_volume_.clear();
  esdf_volume_.clear();
  occupancy_voxels_.clear();
  registration_target_points_.clear();
}

}  // namespace plan_env
