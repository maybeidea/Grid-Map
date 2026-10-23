#include "plan_env/grid_map.hpp"
#include "utils/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace plan_env {

using detail::cleanPolygon;

GridMap::GridMap()
    : tf_buffer_(std::make_shared<tf2_ros::Buffer>(
          std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      latest_esdf_snapshot_(std::make_shared<EsdfSnapshot>()) {
  grid_geometry_.resolution = resolution_;
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
                        const float expansion_margin) {
  return configureDualResolution(resolution, low_resolution_, map_size_z,
                                 expansion_margin);
}

bool GridMap::configureDualResolution(const float high_resolution,
                                      const float low_resolution,
                                      const float map_size_z,
                                      const float expansion_margin) {
  if (!std::isfinite(high_resolution) || high_resolution <= 0.0f ||
      !std::isfinite(low_resolution) || low_resolution <= 0.0f ||
      !std::isfinite(map_size_z) || map_size_z <= 0.0f ||
      !std::isfinite(expansion_margin) || expansion_margin < 0.0f) {
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  resolution_ = high_resolution;
  low_resolution_ = low_resolution;
  grid_geometry_.resolution = high_resolution;
  map_size_z_ = map_size_z;
  expansion_margin_ = expansion_margin;
  return true;
}

bool GridMap::configureOccupancy(const occupancy::LogOddsConfig &config) {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  return occupancy_model_.configure(config);
}

bool GridMap::configureLowOccupancy(const occupancy::LogOddsConfig &config) {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  return low_occupancy_model_.configure(config);
}

bool GridMap::configureEsdf(const esdf::EsdfConfig &config) {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  // Validate without changing the active volume. The volume is configured
  // with this saved policy when the next task ROI is started.
  if (!std::isfinite(config.max_distance) || config.max_distance < 0.0f ||
      std::isnan(config.unknown_z_min) || std::isnan(config.unknown_z_max) ||
      config.unknown_z_min > config.unknown_z_max || config.omp_threads < 1) {
    return false;
  }
  esdf_config_ = config;
  return true;
}

bool GridMap::configureLowEsdf(const esdf::EsdfConfig &config) {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid || !std::isfinite(config.max_distance) ||
      config.max_distance < 0.0f || std::isnan(config.unknown_z_min) ||
      std::isnan(config.unknown_z_max) ||
      config.unknown_z_min > config.unknown_z_max || config.omp_threads < 1) {
    return false;
  }
  low_esdf_config_ = config;
  return true;
}

bool GridMap::configureTsdf(const bool enabled,
                            const float truncation_distance) {
  if (!std::isfinite(truncation_distance) || truncation_distance <= 0.0f) {
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  tsdf_enabled_ = enabled;
  tsdf_truncation_distance_ = truncation_distance;
  return true;
}

bool GridMap::configureLowTsdf(const bool enabled,
                               const float truncation_distance) {
  if (!std::isfinite(truncation_distance) || truncation_distance <= 0.0f) {
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  low_tsdf_enabled_ = enabled;
  low_tsdf_truncation_distance_ = truncation_distance;
  return true;
}

bool GridMap::configurePointCloudFilter(const float leaf_size) {
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0f) {
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  if (task_region_.valid) {
    return false;
  }
  cloud_leaf_size_ = leaf_size;
  return true;
}

bool GridMap::configureRegistration(const registration::Config &config) {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
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

  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  const double original_min_x = min_x;
  const double original_min_y = min_y;
  const double original_max_x = max_x;
  const double original_max_y = max_y;
  const double margin = static_cast<double>(expansion_margin_);
  min_x -= margin;
  min_y -= margin;
  max_x += margin;
  max_y += margin;

  const double expanded_width = max_x - min_x;
  const double expanded_height = max_y - min_y;

  invalidateTaskSnapshots();
  task_region_ = TaskRegion{};
  // Activate only after all volumes have been allocated successfully.
  task_region_.min = {static_cast<float>(min_x), static_cast<float>(min_y)};
  task_region_.original_min = {static_cast<float>(original_min_x),
                               static_cast<float>(original_min_y)};
  task_region_.z_min = 0.0f;
  task_region_.z_max = map_size_z_;
  task_region_.expansion_margin = expansion_margin_;
  task_region_.source_polygon = polygon;

  grid_geometry_.resolution = resolution_;
  grid_geometry_.origin = {task_region_.original_min.x(),
                           task_region_.original_min.y(), task_region_.z_min};

  grid_geometry_.size_x =
      std::max(1, static_cast<int>(std::ceil((original_max_x - original_min_x) /
                                             resolution_)));
  grid_geometry_.size_y =
      std::max(1, static_cast<int>(std::ceil((original_max_y - original_min_y) /
                                             resolution_)));
  grid_geometry_.size_z =
      std::max(1, static_cast<int>(std::ceil(map_size_z_ / resolution_)));

  task_region_.original_max = {
      task_region_.original_min.x() + grid_geometry_.size_x * resolution_,
      task_region_.original_min.y() + grid_geometry_.size_y * resolution_};

  low_grid_geometry_.resolution = low_resolution_;
  low_grid_geometry_.origin = {task_region_.min.x(), task_region_.min.y(),
                               task_region_.z_min};
  low_grid_geometry_.size_x = std::max(
      1, static_cast<int>(std::ceil(expanded_width / low_resolution_)));
  low_grid_geometry_.size_y = std::max(
      1, static_cast<int>(std::ceil(expanded_height / low_resolution_)));
  low_grid_geometry_.size_z =
      std::max(1, static_cast<int>(std::ceil(map_size_z_ / low_resolution_)));

  task_region_.max = {low_grid_geometry_.origin.x() +
                          low_grid_geometry_.size_x * low_resolution_,
                      low_grid_geometry_.origin.y() +
                          low_grid_geometry_.size_y * low_resolution_};
  task_region_.original_polygon = {
      {task_region_.original_min.x(), task_region_.original_min.y()},
      {task_region_.original_max.x(), task_region_.original_min.y()},
      {task_region_.original_max.x(), task_region_.original_max.y()},
      {task_region_.original_min.x(), task_region_.original_max.y()}};
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
      stopTaskMapping();
      return false;
    }
  } else {
    tsdf_volume_.clear();
  }
  if (low_tsdf_enabled_) {
    if (!low_tsdf_volume_.configure(low_grid_geometry_,
                                    low_tsdf_truncation_distance_)) {
      stopTaskMapping();
      return false;
    }
  } else {
    low_tsdf_volume_.clear();
  }
  occupancy_voxels_.assign(voxel_count, OccupancyVoxel{});
  const size_t low_voxel_count =
      static_cast<size_t>(low_grid_geometry_.size_x) *
      static_cast<size_t>(low_grid_geometry_.size_y) *
      static_cast<size_t>(low_grid_geometry_.size_z);
  low_occupancy_voxels_.assign(low_voxel_count, OccupancyVoxel{});
  if (!esdf_volume_.configure(grid_geometry_, esdf_config_)) {
    stopTaskMapping();
    return false;
  }
  if (!low_esdf_volume_.configure(low_grid_geometry_, low_esdf_config_)) {
    stopTaskMapping();
    return false;
  }
  cloud_points_.clear();
  task_region_.valid = true;
  setCloudAdmission(true);

  if (message_out) {
    std::ostringstream stream;
    stream << "task ROI started: source_vertices=" << polygon.size()
           << " original_resolution=" << resolution_
           << " expanded_resolution=" << low_resolution_ << " expanded=["
           << task_region_.min.x() << ", " << task_region_.min.y() << "]..["
           << task_region_.max.x() << ", " << task_region_.max.y()
           << "] map_voxels=[" << grid_geometry_.size_x << ", "
           << grid_geometry_.size_y << ", " << grid_geometry_.size_z << "]";
    *message_out = stream.str();
  }

  return true;
}

void GridMap::stopTaskMapping() {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  invalidateTaskSnapshots();
  task_region_ = TaskRegion{};
  clearMappingBuffers();
  resetMotionState();
  grid_geometry_ = GridGeometry{};
  grid_geometry_.resolution = resolution_;
}

bool GridMap::isTaskMappingActive() const {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  return task_region_.valid;
}

TaskRegion GridMap::taskRegion() const {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  return task_region_;
}

bool GridMap::pointInTaskRegion(const float x, const float y,
                                const float z) const {
  std::lock_guard<std::recursive_mutex> lock(task_region_mutex_);
  return task_region_.contains(x, y, z);
}

void GridMap::setCloudAdmission(const bool enabled) {
  // Caller holds task state; the ROS callback only needs this short mailbox
  // lock.
  std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
  accepting_clouds_ = enabled;
  pending_cloud_generation_ = task_generation_;
  if (!enabled) {
    pending_cloud_.reset();
  }
}

void GridMap::invalidateTaskSnapshots() {
  // Caller holds task_region_mutex_; no old frame may enter a new task.
  ++task_generation_;
  occupancy_updates_enabled_ = true;
  setCloudAdmission(false);
  auto empty = std::make_shared<EsdfSnapshot>();
  empty->version = ++snapshot_version_;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  latest_esdf_snapshot_ = empty;
  frozen_esdf_snapshot_.reset();
  esdf_freezing_query_active_ = false;
}

void GridMap::clearMappingBuffers() {
  tsdf_volume_.clear();
  low_tsdf_volume_.clear();
  esdf_volume_.clear();
  low_esdf_volume_.clear();
  occupancy_voxels_.clear();
  low_occupancy_voxels_.clear();
  registration_target_points_.clear();
  local_ray_observations_.clear();
  local_surface_points_.clear();
  cloud_points_.clear();
  global_map_initialized_ = false;
  global_map_ready_for_current_position_ = false;
  local_map_cross_position_ = false;
  global_map_dirty_ = false;
  low_map_dirty_ = false;
  last_esdf_occupancy_states_.clear();
  esdf_initialized_ = false;
  esdf_current_ = false;
}

}  // namespace plan_env
