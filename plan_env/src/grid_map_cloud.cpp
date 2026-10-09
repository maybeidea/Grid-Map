#include "plan_env/grid_map.hpp"

#include "plan_env/ray_casting.hpp"
#include "plan_env/rclcpp_param.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#if __has_include(<tf2/exceptions.hpp>)
#include <tf2/exceptions.hpp>
#else
#include <tf2/exceptions.h>
#endif

namespace plan_env {

namespace {

float probabilityToLogOdds(const double probability, const double fallback) {
  const double p = std::isfinite(probability) && probability > 0.0 &&
                           probability < 1.0
                       ? probability
                       : fallback;
  return static_cast<float>(std::log(p / (1.0 - p)));
}

template <typename T>
T getParameterWithLegacyAlias(const rclcpp::Node::SharedPtr &node,
                              const char *legacy_name,
                              const char *current_name,
                              const T &default_value) {
  if (node && node->has_parameter(legacy_name)) {
    return node->get_parameter(legacy_name).get_value<T>();
  }
  if (node && node->has_parameter(current_name)) {
    return node->get_parameter(current_name).get_value<T>();
  }
  // A legacy YAML override is not visible through has_parameter() until its
  // name is declared, so inspect overrides before declaring the current name.
  if (node) {
    const auto overrides =
        node->get_node_parameters_interface()->get_parameter_overrides();
    if (overrides.find(legacy_name) != overrides.end()) {
      return declare_or_get_parameter(node, legacy_name, default_value);
    }
  }
  // Declare the current spelling. This also picks up a current-name override.
  return declare_or_get_parameter(node, current_name, default_value);
}

}  // namespace

void GridMap::initMap(rclcpp::Node::SharedPtr node) {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  node_ = std::move(node);
  if (!node_) {
    cloud_sub_.reset();
    return;
  }
  const auto parameter = [this](const char *name, auto &value) {
    value = declare_or_get_parameter(node_, name, value);
  };

  // Keep the old plan_env initialization contract. XY dimensions are derived
  // from the task polygon by the new map, while resolution and Z extent keep
  // their original grid_map parameter names.
  const double legacy_resolution =
      declare_or_get_parameter<double>(node_, "grid_map.resolution", 0.05);
  const double legacy_map_size_z =
      declare_or_get_parameter<double>(node_, "grid_map.map_size_z", 0.60);
  const double expansion_margin = declare_or_get_parameter<double>(
      node_, "grid_map.expansion_margin", 0.25);
  const double expansion_ratio = declare_or_get_parameter<double>(
      node_, "grid_map.expansion_ratio", 0.10);
  if (!configure(static_cast<float>(legacy_resolution),
                 static_cast<float>(legacy_map_size_z),
                 static_cast<float>(expansion_margin),
                 static_cast<float>(expansion_ratio))) {
    RCLCPP_WARN(node_->get_logger(),
                "GridMap: invalid geometry parameters; keeping defaults "
                "(resolution=%.3f, map_size_z=%.3f)",
                resolution_, map_size_z_);
  }

  const double p_hit = declare_or_get_parameter<double>(
      node_, "grid_map.p_hit", 0.70);
  const double p_miss = declare_or_get_parameter<double>(
      node_, "grid_map.p_miss", 0.35);
  const double p_min = declare_or_get_parameter<double>(
      node_, "grid_map.p_min", 0.12);
  const double p_max = declare_or_get_parameter<double>(
      node_, "grid_map.p_max", 0.97);
  const double p_occ = declare_or_get_parameter<double>(
      node_, "grid_map.p_occ", 0.80);
  occupancy::LogOddsConfig occupancy_config;
  occupancy_config.hit_log_odds = probabilityToLogOdds(p_hit, 0.70);
  occupancy_config.miss_log_odds = probabilityToLogOdds(p_miss, 0.35);
  occupancy_config.min_log_odds = probabilityToLogOdds(p_min, 0.12);
  occupancy_config.max_log_odds = probabilityToLogOdds(p_max, 0.97);
  occupancy_config.occupied_threshold = probabilityToLogOdds(p_occ, 0.80);
  // The old classifier treated all values between the unknown sentinel and
  // p_occ as known-free. A zero boundary preserves that behavior for misses.
  occupancy_config.free_threshold = 0.0f;
  if (!configureOccupancy(occupancy_config)) {
    RCLCPP_WARN(node_->get_logger(),
                "GridMap: invalid occupancy probability parameters; using defaults");
    configureOccupancy(occupancy::LogOddsConfig{});
  }

  const std::string unknown_mode = declare_or_get_parameter<std::string>(
      node_, "grid_map.esdf_unknown_mode", "task_roi");
  esdf::EsdfConfig esdf_config;
  esdf_config.unknown_is_occupied = unknown_mode != "off";
  const double roi_z_max = declare_or_get_parameter<double>(
      node_, "grid_map.esdf_roi_z_max", 0.50);
  const double ground_height = declare_or_get_parameter<double>(
      node_, "grid_map.ground_height", 0.0);
  if (unknown_mode != "all") {
    esdf_config.unknown_z_min = static_cast<float>(ground_height);
    esdf_config.unknown_z_max = static_cast<float>(roi_z_max);
  }
  if (unknown_mode != "off" && unknown_mode != "task_roi" && unknown_mode != "all") {
    RCLCPP_WARN(node_->get_logger(), "GridMap: unknown esdf_unknown_mode '%s'; using task_roi",
                unknown_mode.c_str());
  }
  esdf_config.signed_distance = declare_or_get_parameter<bool>(
      node_, "grid_map.esdf_signed_distance", true);
  const double esdf_max_distance = declare_or_get_parameter<double>(
      node_, "grid_map.esdf_max_distance", 100000.0);
  esdf_config.max_distance = static_cast<float>(esdf_max_distance);
  if (!configureEsdf(esdf_config)) {
    RCLCPP_WARN(node_->get_logger(),
                "GridMap: invalid ESDF parameters; using defaults");
    configureEsdf(esdf::EsdfConfig{});
  }

  parameter("grid_map.cloud_min_map_z", cloud_min_map_z_);
  parameter("grid_map.cloud_low_z_clear_rays", cloud_low_z_clear_rays_);
  parameter("grid_map.cloud_min_range", cloud_min_range_);
  parameter("grid_map.local_update_range_x", local_update_range_.x());
  parameter("grid_map.local_update_range_y", local_update_range_.y());
  parameter("grid_map.local_update_range_z", local_update_range_.z());
  if (!std::isfinite(cloud_min_map_z_)) cloud_min_map_z_ = 0.02;
  if (!std::isfinite(cloud_min_range_) || cloud_min_range_ < 0.0) cloud_min_range_ = 0.2;
  if (!local_update_range_.allFinite() || (local_update_range_.array() <= 0.0f).any()) {
    local_update_range_ = Eigen::Vector3f(5.0f, 5.0f, 3.0f);
  }
  parameter("grid_map.motion_frame", motion_frame_);
  parameter("grid_map.frame_id", map_frame_);
  parameter("grid_map.motion_translation_threshold",
            motion_translation_threshold_);
  parameter("grid_map.motion_rotation_threshold_rad",
            motion_rotation_threshold_rad_);
  int stationary_frames = static_cast<int>(stationary_frames_required_);
  parameter("grid_map.stationary_frames", stationary_frames);
  if (stationary_frames > 0) {
    stationary_frames_required_ = static_cast<uint32_t>(stationary_frames);
  }
  registration::Config registration_config = registration_backend_.config();
  parameter("grid_map.registration.enabled", registration_config.enabled);
  parameter("grid_map.registration.max_correspondence_distance",
            registration_config.max_correspondence_distance);
  parameter("grid_map.registration.maximum_iterations",
            registration_config.maximum_iterations);
  parameter("grid_map.registration.min_overlap",
            registration_config.min_overlap);
  parameter("grid_map.registration.max_fitness",
            registration_config.max_fitness);
  parameter("grid_map.registration.max_translation_correction",
            registration_config.max_translation_correction);
  parameter("grid_map.registration.max_rotation_correction_rad",
            registration_config.max_rotation_correction_rad);
  int min_target_points =
      static_cast<int>(registration_config.min_target_points);
  parameter("grid_map.registration.min_target_points", min_target_points);
  if (min_target_points > 0) {
    registration_config.min_target_points =
        static_cast<std::size_t>(min_target_points);
  }
  if (!registration_backend_.configure(registration_config)) {
    registration_backend_.configure(registration::Config{});
  }
  bool tsdf_enabled = tsdf_enabled_;
  float tsdf_truncation_distance = tsdf_truncation_distance_;
  parameter("grid_map.tsdf.enabled", tsdf_enabled);
  parameter("grid_map.tsdf.truncation_distance", tsdf_truncation_distance);
  if (!configureTsdf(tsdf_enabled, tsdf_truncation_distance)) {
    tsdf_enabled_ = false;
    tsdf_truncation_distance_ = 0.05f;
  }
  float cloud_leaf_size = static_cast<float>(getParameterWithLegacyAlias(
      node_, "grid_map.cloud_leaf_size", "grid_map.point_cloud_leaf_size",
      static_cast<double>(cloud_leaf_size_)));
  if (!configurePointCloudFilter(cloud_leaf_size)) {
    cloud_leaf_size_ = 0.005f;
  }
  parameter("grid_map.height_map_enable", height_map_enable_);
  parameter("grid_map.height_map_topic", height_map_topic_);
  parameter("grid_map.height_map_frame", height_map_frame_);
  parameter("grid_map.height_band_z_max", height_band_z_max_);
  parameter("grid_map.height_completeness_z_max", height_completeness_z_max_);
  parameter("grid_map.height_floor_tol", height_floor_tol_);
  parameter("grid_map.height_vision_limit", height_vision_limit_);
  parameter("grid_map.height_z_tcp_min", height_z_tcp_min_);
  parameter("grid_map.height_ceil_threshold_open", height_ceil_threshold_open_);
  if (!std::isfinite(motion_translation_threshold_) ||
      motion_translation_threshold_ < 0.0f) {
    motion_translation_threshold_ = 0.03f;
  }
  if (!std::isfinite(motion_rotation_threshold_rad_) ||
      motion_rotation_threshold_rad_ < 0.0f) {
    motion_rotation_threshold_rad_ = 0.02f;
  }
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", rclcpp::SensorDataQoS(),
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

bool GridMap::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  if (!cloud) return false;

  {
    std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
    if (stopping_ || !accepting_clouds_) {
      return false;
    }
    pending_cloud_ = cloud;
  }
  pending_cloud_cv_.notify_one();
  return true;
}

void GridMap::mappingLoop() {
  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    uint64_t generation = 0;
    {
      std::unique_lock<std::mutex> lock(pending_cloud_mutex_);
      pending_cloud_cv_.wait(lock,
                             [this] { return stopping_ || pending_cloud_; });
      if (stopping_) {
        return;
      }
      cloud = std::move(pending_cloud_);
      generation = pending_cloud_generation_;
      pending_cloud_.reset();
    }

    processQueuedCloud(cloud, generation);
  }
}

bool GridMap::processQueuedCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
    const uint64_t generation) {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  if (generation != task_generation_ || !task_region_.valid ||
      !occupancy_updates_enabled_ || !processPointCloudFrame(cloud)) {
    return false;
  }
  // Publish only complete stationary positions, within the task transaction.
  publishEsdfSnapshot();
  publishHeightMap();
  return true;
}

bool GridMap::processPointCloudFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  if (!task_region_.valid || !occupancy_updates_enabled_) return false;
  PreparedCloud prepared;
  if (!processPointCloud(cloud, &prepared)) {
    return false;
  }

  FusionMode mode = FusionMode::kSamePosition;
  if (!updateMotionState(prepared, &mode)) {
    // A cloud can still be transformed while the robot is moving, but it is
    // intentionally not committed to cloud_points_ or to any map layer.
    return false;
  }

  Eigen::Isometry3f pose = station_pose_correction_ * prepared.prior_pose;
  if (mode == FusionMode::kDifferentPosition &&
      !registerDifferentPosition(prepared, &pose)) {
    return false;
  }

  std::vector<Eigen::Vector3f> transformed_points;
  transformed_points.reserve(prepared.sensor_points.size());
  const TaskRegion region = taskRegion();
  for (const auto &point : prepared.sensor_points) {
    const Eigen::Vector3f point_map = pose * point;
    const bool clearing_ground = cloud_low_z_clear_rays_ && point_map.z() < cloud_min_map_z_;
    if (region.valid &&
        !region.contains(point_map.x(), point_map.y(),
                         clearing_ground ? std::max(point_map.z(), region.z_min) : point_map.z())) {
      continue;
    }
    if (point_map.z() < cloud_min_map_z_ && !cloud_low_z_clear_rays_) {
      continue;
    }
    transformed_points.push_back(point_map);
  }
  if (transformed_points.empty()) {
    return false;
  }
  cloud_points_ = std::move(transformed_points);
  camera_position_ = pose.translation();
  const bool fused = mode == FusionMode::kSamePosition
                         ? fuseSamePositionFrame()
                         : fuseDifferentPositionFrame();
  if (!fused) return false;
  registration_target_points_.insert(registration_target_points_.end(),
                                     cloud_points_.begin(),
                                     cloud_points_.end());
  constexpr std::size_t kMaxRegistrationTargetPoints = 100000;
  if (registration_target_points_.size() > kMaxRegistrationTargetPoints) {
    const std::size_t excess =
        registration_target_points_.size() - kMaxRegistrationTargetPoints;
    registration_target_points_.erase(registration_target_points_.begin(),
                                      registration_target_points_.begin() +
                                          static_cast<std::ptrdiff_t>(excess));
  }

  // Apply the accepted registration consistently to every frame at this stop.
  station_pose_correction_ = pose * prepared.prior_pose.inverse();
  if (!recordSuccessfulFusion()) return false;

  // The distance field is intentionally recomputed only after the current
  // stationary position has accumulated its successful fusion quota.
  if (!esdf_volume_.compute(occupancy_voxels_, occupancy_model_)) return false;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_state_ = MotionState::kReadyToMove;
  }
  return true;
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
    return false;
  }

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
        point_camera.norm() <= cloud_min_range_) {
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
    } catch (const tf2::TransformException &) {
      // Keep the cloud-frame pose fallback. It preserves operation for
      // systems that only publish a camera TF tree.
    }
  }
  return true;
}

bool GridMap::fuseSamePositionFrame() {
  // Temporal fusion at one stationary pose.
  if (!updateMapsFromRays()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    ++same_position_fused_frames_;
  }
  return true;
}

bool GridMap::fuseDifferentPositionFrame() {
  // Cross-position fusion uses the same integrator after registration.
  if (!updateMapsFromRays()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    ++different_position_fused_frames_;
  }
  return true;
}

bool GridMap::updateMapsFromRays() {
  if (grid_geometry_.resolution <= 0.0f || occupancy_voxels_.empty()) {
    return false;
  }
  const Eigen::Vector3i grid_size(grid_geometry_.size_x, grid_geometry_.size_y,
                                  grid_geometry_.size_z);
  const Eigen::Vector3f ray_origin = camera_position_;
  bool integrated_voxel = false;
  for (const auto &point : cloud_points_) {
    const bool low_z = point.z() < cloud_min_map_z_;
    if (low_z && !cloud_low_z_clear_rays_) continue;
    const bool endpoint_in_grid = ray_casting::pointInGrid(
        grid_geometry_.origin, grid_geometry_.resolution, grid_size, point);
    ray_casting::traverseRay(
        grid_geometry_.origin, grid_geometry_.resolution, grid_size, ray_origin,
        point,
        [this, &ray_origin, &point, &integrated_voxel,
         endpoint_in_grid, low_z](const Eigen::Vector3i &voxel_id, bool hit) {
          const int address = toAddress(voxel_id);
          if (address < 0 ||
              static_cast<size_t>(address) >= occupancy_voxels_.size()) {
            return;
          }
          integrated_voxel = true;
          occupancy_model_.update(occupancy_voxels_[address], hit && !low_z);
          if (tsdf_enabled_ && endpoint_in_grid && !low_z) {
            tsdf_volume_.integrateVoxel(voxel_id, ray_origin, point);
          }
        });
  }
  return integrated_voxel;
}

}  // namespace plan_env
