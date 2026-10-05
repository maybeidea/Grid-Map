#include "plan_env/grid_map.hpp"

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
#include <utility>
#if __has_include(<tf2/exceptions.h>)
#include <tf2/exceptions.h>
#else
#include <tf2/exceptions.hpp>

namespace plan_env {
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

void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) const {
  if (grid_geometry_.resolution <= 0.0f) {
    id.setConstant(INVALID_IDX);
    return;
  }
  id = ((pos - grid_geometry_.origin.cast<double>()) /
        static_cast<double>(grid_geometry_.resolution)).array().floor().cast<int>();
}

bool GridMap::isInMap(const Eigen::Vector3i &id) const {
  return id.x() >= 0 && id.y() >= 0 && id.z() >= 0 &&
         id.x() < grid_geometry_.size_x && id.y() < grid_geometry_.size_y &&
         id.z() < grid_geometry_.size_z;
}

double GridMap::getResolution() const {
  return static_cast<double>(grid_geometry_.resolution);
}

void GridMap::getRegion(Eigen::Vector3d &origin, Eigen::Vector3d &size) const {
  origin = grid_geometry_.origin.cast<double>();
  size = Eigen::Vector3d(grid_geometry_.size_x, grid_geometry_.size_y,
                         grid_geometry_.size_z) * getResolution();
}

void GridMap::initMap(rclcpp::Node::SharedPtr node) {
  node_ = std::move(node);
  if (!node_) {
    cloud_sub_.reset();
    return;
  }
  // Keep the gate configurable without requiring callers to change the API.
  // get_parameter_or also works when a launch file did not provide overrides.
  node_->get_parameter_or("grid_map.motion_frame", motion_frame_, motion_frame_);
  node_->get_parameter_or("grid_map.frame_id", map_frame_, map_frame_);
  node_->get_parameter_or("grid_map.motion_translation_threshold",
                          motion_translation_threshold_,
                          motion_translation_threshold_);
  node_->get_parameter_or("grid_map.motion_rotation_threshold_rad",
                          motion_rotation_threshold_rad_,
                          motion_rotation_threshold_rad_);
  int stationary_frames = static_cast<int>(stationary_frames_required_);
  node_->get_parameter_or("grid_map.stationary_frames", stationary_frames,
                          stationary_frames);
  if (stationary_frames > 0) {
    stationary_frames_required_ = static_cast<uint32_t>(stationary_frames);
  }
  registration::Config registration_config = registration_backend_.config();
  node_->get_parameter_or("grid_map.registration.enabled",
                          registration_config.enabled,
                          registration_config.enabled);
  node_->get_parameter_or("grid_map.registration.max_correspondence_distance",
                          registration_config.max_correspondence_distance,
                          registration_config.max_correspondence_distance);
  node_->get_parameter_or("grid_map.registration.maximum_iterations",
                          registration_config.maximum_iterations,
                          registration_config.maximum_iterations);
  node_->get_parameter_or("grid_map.registration.min_overlap",
                          registration_config.min_overlap,
                          registration_config.min_overlap);
  node_->get_parameter_or("grid_map.registration.max_fitness",
                          registration_config.max_fitness,
                          registration_config.max_fitness);
  node_->get_parameter_or("grid_map.registration.max_translation_correction",
                          registration_config.max_translation_correction,
                          registration_config.max_translation_correction);
  node_->get_parameter_or("grid_map.registration.max_rotation_correction_rad",
                          registration_config.max_rotation_correction_rad,
                          registration_config.max_rotation_correction_rad);
  int min_target_points = static_cast<int>(registration_config.min_target_points);
  node_->get_parameter_or("grid_map.registration.min_target_points",
                          min_target_points, min_target_points);
  if (min_target_points > 0) {
    registration_config.min_target_points =
        static_cast<std::size_t>(min_target_points);
  }
  if (!registration_backend_.configure(registration_config)) {
    registration_backend_.configure(registration::Config{});
  }
  bool tsdf_enabled = tsdf_enabled_;
  float tsdf_truncation_distance = tsdf_truncation_distance_;
  node_->get_parameter_or("grid_map.tsdf.enabled", tsdf_enabled,
                          tsdf_enabled);
  node_->get_parameter_or("grid_map.tsdf.truncation_distance",
                          tsdf_truncation_distance,
                          tsdf_truncation_distance);
  if (!configureTsdf(tsdf_enabled, tsdf_truncation_distance)) {
    tsdf_enabled_ = false;
    tsdf_truncation_distance_ = 0.05f;
  }
  float cloud_leaf_size = cloud_leaf_size_;
  node_->get_parameter_or("grid_map.point_cloud_leaf_size", cloud_leaf_size,
                          cloud_leaf_size);
  if (!configurePointCloudFilter(cloud_leaf_size)) {
    cloud_leaf_size_ = 0.005f;
  }
  node_->get_parameter_or("grid_map.height_map_enable", height_map_enable_,
                          height_map_enable_);
  node_->get_parameter_or("grid_map.height_map_topic", height_map_topic_,
                          height_map_topic_);
  node_->get_parameter_or("grid_map.height_map_frame", height_map_frame_,
                          height_map_frame_);
  node_->get_parameter_or("grid_map.height_band_z_max", height_band_z_max_,
                          height_band_z_max_);
  node_->get_parameter_or("grid_map.height_completeness_z_max",
                          height_completeness_z_max_, height_completeness_z_max_);
  node_->get_parameter_or("grid_map.height_floor_tol", height_floor_tol_,
                          height_floor_tol_);
  node_->get_parameter_or("grid_map.height_blocked_at_floor_threshold",
                          height_blocked_at_floor_threshold_,
                          height_blocked_at_floor_threshold_);
  node_->get_parameter_or("grid_map.height_z_tcp_min", height_z_tcp_min_,
                          height_z_tcp_min_);
  node_->get_parameter_or("grid_map.height_ceil_threshold_open",
                          height_ceil_threshold_open_, height_ceil_threshold_open_);
  if (!std::isfinite(motion_translation_threshold_) ||
      motion_translation_threshold_ < 0.0f) {
    motion_translation_threshold_ = 0.03f;
  }
  if (!std::isfinite(motion_rotation_threshold_rad_) ||
      motion_rotation_threshold_rad_ < 0.0f) {
    motion_rotation_threshold_rad_ = 0.02f;
  }
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        pointCloudCallback(cloud);
      });
  #ifdef PLAN_ENV_HAS_BIMAX_MSGS
  if (height_map_enable_) {
    height_map_pub_ = node_->create_publisher<bimax_msgs::msg::HeightMap>(
        height_map_topic_, rclcpp::QoS(10));
  }
  #endif
}

bool GridMap::configureMotion(const float translation_threshold,
                              const float rotation_threshold_rad,
                              const uint32_t stationary_frames) {
  if (!std::isfinite(translation_threshold) || translation_threshold < 0.0f ||
      !std::isfinite(rotation_threshold_rad) || rotation_threshold_rad < 0.0f ||
      stationary_frames == 0) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_translation_threshold_ = translation_threshold;
    motion_rotation_threshold_rad_ = rotation_threshold_rad;
    stationary_frames_required_ = stationary_frames;
  }
  resetMotionState();
  return true;
}

MappingStatus GridMap::mappingStatus() const {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  MappingStatus status;
  status.motion_state = motion_state_;
  status.last_fusion_mode = last_fusion_mode_;
  status.stationary_frame_count = stationary_frame_count_;
  status.stationary_frames_required = stationary_frames_required_;
  status.position_index = position_index_;
  status.same_position_fused_frames = same_position_fused_frames_;
  status.different_position_fused_frames = different_position_fused_frames_;
  status.ready_to_move = motion_state_ == MotionState::kReadyToMove;
  return status;
}

bool GridMap::readyToMove() const {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  return motion_state_ == MotionState::kReadyToMove;
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

bool GridMap::startTaskMapping(
    const std::vector<Eigen::Vector2d>& polygon_xy,
    std::string* message_out) {

  const auto polygon = cleanPolygon(polygon_xy);
  if (polygon.size() < 3) {
    if (message_out) {
      *message_out =
          "task polygon needs at least 3 finite unique vertices";
    }
    return false;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto& p : polygon) {
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
  task_region_.min = {
      static_cast<float>(min_x),
      static_cast<float>(min_y)};
  task_region_.z_min = 0.0f;
  task_region_.z_max = map_size_z_;
  task_region_.expansion_margin = expansion_margin_;
  task_region_.source_polygon = polygon;

  grid_geometry_.resolution = resolution_;
  grid_geometry_.origin = {
      task_region_.min.x(),
      task_region_.min.y(),
      task_region_.z_min};

  grid_geometry_.size_x = std::max(
      1, static_cast<int>(std::ceil(width / resolution_)));
  grid_geometry_.size_y = std::max(
      1, static_cast<int>(std::ceil(height / resolution_)));
  grid_geometry_.size_z = std::max(
      1, static_cast<int>(std::ceil(map_size_z_ / resolution_)));

  // Use the actual voxel-aligned map boundary.
  task_region_.max = {
      task_region_.min.x() +
          grid_geometry_.size_x * resolution_,
      task_region_.min.y() +
          grid_geometry_.size_y * resolution_};

  task_region_.expanded_polygon = {
      {task_region_.min.x(), task_region_.min.y()},
      {task_region_.max.x(), task_region_.min.y()},
      {task_region_.max.x(), task_region_.max.y()},
      {task_region_.min.x(), task_region_.max.y()}};

  clearMappingBuffers();
  resetMotionState();

  const size_t voxel_count =
      static_cast<size_t>(grid_geometry_.size_x) *
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
           << " expanded=[" << task_region_.min.x()
           << ", " << task_region_.min.y()
           << "]..[" << task_region_.max.x()
           << ", " << task_region_.max.y()
           << "] map_voxels=["
           << grid_geometry_.size_x << ", "
           << grid_geometry_.size_y << ", "
           << grid_geometry_.size_z << "]";
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

bool GridMap::pointInTaskRegion(const float x, const float y, const float z) const {
  std::lock_guard<std::mutex> lock(task_region_mutex_);
  return task_region_.contains(x, y, z);
}

void GridMap::clearMappingBuffers() {
  tsdf_volume_.clear();
  esdf_volume_.clear();
  occupancy_voxels_.clear();
  registration_target_points_.clear();
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

    processPointCloudFrame(cloud);
    publishEsdfSnapshot();
    publishHeightMap();
  }
}

void GridMap::processPointCloudFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  PreparedCloud prepared;
  if (!processPointCloud(cloud, &prepared)) {
    return;
  }

  FusionMode mode = FusionMode::kSamePosition;
  if (!updateMotionState(prepared, &mode)) {
    // A cloud can still be transformed while the robot is moving, but it is
    // intentionally not committed to cloud_points_ or to any map layer.
    return;
  }

  Eigen::Isometry3f pose = prepared.prior_pose;
  if (mode == FusionMode::kDifferentPosition &&
      !registerDifferentPosition(prepared, &pose)) {
    return;
  }

  std::vector<Eigen::Vector3f> transformed_points;
  transformed_points.reserve(prepared.sensor_points.size());
  const TaskRegion region = taskRegion();
  for (const auto &point : prepared.sensor_points) {
    const Eigen::Vector3f point_map = pose * point;
    if (region.valid &&
        !region.contains(point_map.x(), point_map.y(), point_map.z())) {
      continue;
    }
    transformed_points.push_back(point_map);
  }
  if (transformed_points.empty()) {
    return;
  }
  cloud_points_ = std::move(transformed_points);
  camera_position_ = pose.translation();
  registration_target_points_.insert(registration_target_points_.end(),
                                     cloud_points_.begin(), cloud_points_.end());
  constexpr std::size_t kMaxRegistrationTargetPoints = 100000;
  if (registration_target_points_.size() > kMaxRegistrationTargetPoints) {
    const std::size_t excess = registration_target_points_.size() -
                               kMaxRegistrationTargetPoints;
    registration_target_points_.erase(registration_target_points_.begin(),
                                      registration_target_points_.begin() +
                                          static_cast<std::ptrdiff_t>(excess));
  }
  if (mode == FusionMode::kSamePosition) {
    fuseSamePositionFrame();
  } else {
    fuseDifferentPositionFrame();
  }
}

bool GridMap::registerDifferentPosition(const PreparedCloud &prepared,
                                        Eigen::Isometry3f *pose) {
  if (!pose || !registration_backend_.config().enabled ||
      registration_target_points_.size() <
                   registration_backend_.config().min_target_points) {
    return true;
  }
  const registration::Result result = registration_backend_.align(
      prepared.sensor_points, registration_target_points_, prepared.prior_pose);
  if (!result.accepted) {
    return false;
  }
  *pose = result.pose;
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
    return false;
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
    return false;
  }

  constexpr float kCloudMinRange = 0.2f;
  const Eigen::Vector3f local_update_range(5.0f, 5.0f, 3.0f);
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(finite_cloud);
  voxel_filter.setLeafSize(cloud_leaf_size_, cloud_leaf_size_,
                           cloud_leaf_size_);
  voxel_filter.filter(filtered_cloud);
  if (filtered_cloud.empty()) {
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
  } catch (const tf2::TransformException &) {
    return false;
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

  std::vector<Eigen::Vector3f> sensor_points;
  sensor_points.reserve(filtered_cloud.points.size());
  for (const auto &point : filtered_cloud.points) {
    const Eigen::Vector3f point_camera(point.x, point.y, point.z);
    if (std::abs(point_camera.x()) >= local_update_range.x() ||
        std::abs(point_camera.y()) >= local_update_range.y() ||
        std::abs(point_camera.z()) >= local_update_range.z() ||
        point_camera.norm() <= kCloudMinRange) {
      continue;
    }
    sensor_points.push_back(point_camera);
  }
  if (sensor_points.empty()) {
    return false;
  }

  prepared->sensor_points = std::move(sensor_points);
  prepared->prior_pose = Eigen::Isometry3f::Identity();
  prepared->prior_pose.linear() = rotation_matrix;
  prepared->prior_pose.translation() = translation_vector;
  prepared->camera_position = translation_vector;

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
      prepared->motion_position = Eigen::Vector3f(
          static_cast<float>(motion_translation.x),
          static_cast<float>(motion_translation.y),
          static_cast<float>(motion_translation.z));
      prepared->motion_orientation = Eigen::Quaternionf(
          static_cast<float>(motion_rotation.w),
          static_cast<float>(motion_rotation.x),
          static_cast<float>(motion_rotation.y),
          static_cast<float>(motion_rotation.z)).normalized();
    } catch (const tf2::TransformException &) {
      // Keep the cloud-frame pose fallback. It preserves operation for
      // systems that only publish a camera TF tree.
    }
  }
  return true;
}

void GridMap::resetMotionState() {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  motion_state_ = MotionState::kWaitingForPose;
  last_fusion_mode_ = FusionMode::kSamePosition;
  last_motion_position_.setZero();
  last_motion_orientation_ = Eigen::Quaternionf::Identity();
  stationary_anchor_position_.setZero();
  stationary_anchor_orientation_ = Eigen::Quaternionf::Identity();
  has_last_motion_pose_ = false;
  has_stationary_anchor_ = false;
  awaiting_new_position_ = false;
  stationary_frame_count_ = 0;
  position_index_ = 0;
  same_position_fused_frames_ = 0;
  different_position_fused_frames_ = 0;
}

bool GridMap::updateMotionState(const PreparedCloud &prepared, FusionMode *mode) {
  if (!mode || !prepared.motion_position.allFinite() ||
      !prepared.motion_orientation.coeffs().allFinite()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(motion_mutex_);
  if (!has_last_motion_pose_) {
    last_motion_position_ = prepared.motion_position;
    last_motion_orientation_ = prepared.motion_orientation;
    has_last_motion_pose_ = true;
    motion_state_ = MotionState::kWaitingForPose;
    return false;
  }

  const float translation_delta =
      (prepared.motion_position - last_motion_position_).norm();
  const float rotation_delta =
      Eigen::AngleAxisf(last_motion_orientation_.conjugate() *
                            prepared.motion_orientation)
          .angle();
  const float anchor_translation_delta =
      has_stationary_anchor_
          ? (prepared.motion_position - stationary_anchor_position_).norm()
          : 0.0f;
  const float anchor_rotation_delta =
      has_stationary_anchor_
          ? Eigen::AngleAxisf(stationary_anchor_orientation_.conjugate() *
                                  prepared.motion_orientation)
                .angle()
          : 0.0f;
  last_motion_position_ = prepared.motion_position;
  last_motion_orientation_ = prepared.motion_orientation;

  if (translation_delta > motion_translation_threshold_ ||
      rotation_delta > motion_rotation_threshold_rad_ ||
      anchor_translation_delta > motion_translation_threshold_ ||
      anchor_rotation_delta > motion_rotation_threshold_rad_) {
    motion_state_ = MotionState::kMoving;
    stationary_frame_count_ = 0;
    awaiting_new_position_ = true;
    return false;
  }

  // Once 30 frames have been fused, stop accepting additional frames at the
  // same stop. This is the gate that tells the planner it may command motion.
  if (motion_state_ == MotionState::kReadyToMove) {
    return false;
  }

  const bool new_position = !has_stationary_anchor_ || awaiting_new_position_;
  if (new_position) {
    stationary_anchor_position_ = prepared.motion_position;
    stationary_anchor_orientation_ = prepared.motion_orientation;
    has_stationary_anchor_ = true;
    awaiting_new_position_ = false;
    stationary_frame_count_ = 0;
    ++position_index_;
    *mode = position_index_ > 1 ? FusionMode::kDifferentPosition
                                : FusionMode::kSamePosition;
  } else {
    *mode = FusionMode::kSamePosition;
  }

  ++stationary_frame_count_;
  motion_state_ = stationary_frame_count_ >= stationary_frames_required_
                      ? MotionState::kReadyToMove
                      : MotionState::kStationaryCollecting;
  last_fusion_mode_ = *mode;
  return true;
}

void GridMap::fuseSamePositionFrame() {
  // Placeholder policy boundary: temporal fusion at one stationary pose.
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    ++same_position_fused_frames_;
  }
  updateMapsFromRays();
}

void GridMap::fuseDifferentPositionFrame() {
  // Placeholder policy boundary: merge a completed pose into the world map.
  // This is intentionally separate from same-position temporal fusion so the
  // two algorithms can evolve independently later.
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    ++different_position_fused_frames_;
  }
  updateMapsFromRays();
}

void GridMap::updateMapsFromRays() {
  if (grid_geometry_.resolution <= 0.0f || occupancy_voxels_.empty()) {
    return;
  }
  const Eigen::Vector3i grid_size(grid_geometry_.size_x,
                                  grid_geometry_.size_y,
                                  grid_geometry_.size_z);
  const Eigen::Vector3f ray_origin = camera_position_;
  for (const auto &point : cloud_points_) {
    const bool endpoint_in_grid = ray_casting::pointInGrid(
        grid_geometry_.origin, grid_geometry_.resolution, grid_size, point);
    ray_casting::traverseRay(
        grid_geometry_.origin, grid_geometry_.resolution, grid_size,
        ray_origin, point,
        [this, &ray_origin, &point, endpoint_in_grid](
            const Eigen::Vector3i &voxel_id, bool hit) {
          const int address = toAddress(voxel_id);
          if (address < 0 ||
              static_cast<size_t>(address) >= occupancy_voxels_.size()) {
            return;
          }
          updateOccupancyVoxel(occupancy_voxels_[address], hit);
          if (tsdf_enabled_ && endpoint_in_grid) {
            tsdf_volume_.integrateVoxel(voxel_id, ray_origin, point);
          }
        });
  }
  updateEsdf();
}

void GridMap::updateOccupancyVoxel(OccupancyVoxel &voxel, const bool hit) {
  occupancy_model_.update(voxel, hit);
}

void GridMap::updateEsdf() {
  esdf_volume_.compute(occupancy_voxels_, occupancy_model_);
}

void GridMap::publishEsdfSnapshot() {
  auto next = std::make_shared<EsdfSnapshot>();
  next->geometry = grid_geometry_;
  next->tsdf_voxels = tsdf_volume_.voxels();
  next->occupancy_voxels = occupancy_voxels_;
  next->occupancy_probabilities.resize(occupancy_voxels_.size());
  next->occupancy_states.resize(occupancy_voxels_.size());
  for (size_t i = 0; i < occupancy_voxels_.size(); ++i) {
    next->occupancy_probabilities[i] =
        occupancy_model_.probability(occupancy_voxels_[i]);
    next->occupancy_states[i] = occupancy_model_.state(occupancy_voxels_[i]);
  }
  next->esdf_voxels = esdf_volume_.voxels();
  next->camera_position = camera_position_;
  next->tsdf_enabled = tsdf_enabled_;
  const TaskRegion region = taskRegion();
  next->task_mapping_active = region.valid;
  next->task_roi_polygon = region.expanded_polygon;
  next->mapping_status = mappingStatus();
  next->version = ++snapshot_version_;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_esdf_snapshot_ = std::shared_ptr<const EsdfSnapshot>(std::move(next));
  }
}

#ifndef PLAN_ENV_HAS_BIMAX_MSGS
bool GridMap::buildHeightMapMsg(
    bimax_msgs::msg::HeightMap &, const Eigen::Isometry3d &) {
  return false;
}

void GridMap::publishHeightMap() {}
#endif

}  // namespace plan_env
