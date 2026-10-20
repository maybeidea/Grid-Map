#include "plan_env/grid_map.hpp"
#include "plan_env/rclcpp_param.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace plan_env {

namespace {

float probabilityToLogOdds(const double probability, const double fallback) {
  const double p =
      std::isfinite(probability) && probability > 0.0 && probability < 1.0
          ? probability
          : fallback;
  return static_cast<float>(std::log(p / (1.0 - p)));
}

struct OccupancyProbabilities {
  double hit = 0.70, miss = 0.35, min = 0.12, max = 0.97, occupied = 0.80;

  occupancy::LogOddsConfig config() const {
    occupancy::LogOddsConfig result;
    result.hit_log_odds = probabilityToLogOdds(hit, 0.70);
    result.miss_log_odds = probabilityToLogOdds(miss, 0.35);
    result.min_log_odds = probabilityToLogOdds(min, 0.12);
    result.max_log_odds = probabilityToLogOdds(max, 0.97);
    result.occupied_threshold = probabilityToLogOdds(occupied, 0.80);
    result.free_threshold = 0.0f;
    return result;
  }
};

OccupancyProbabilities readOccupancy(const rclcpp::Node::SharedPtr &node,
                                     const std::string &prefix,
                                     OccupancyProbabilities values) {
  values.hit = declare_or_get_parameter(node, prefix + "p_hit", values.hit);
  values.miss = declare_or_get_parameter(node, prefix + "p_miss", values.miss);
  values.min = declare_or_get_parameter(node, prefix + "p_min", values.min);
  values.max = declare_or_get_parameter(node, prefix + "p_max", values.max);
  values.occupied =
      declare_or_get_parameter(node, prefix + "p_occ", values.occupied);
  return values;
}

struct EsdfParameters {
  std::string unknown_mode = "task_roi";
  double roi_z_max = 0.50;
  double max_distance = 100000.0;
  bool signed_distance = true;

  esdf::EsdfConfig config(double ground_height) const {
    esdf::EsdfConfig result;
    result.unknown_is_occupied = unknown_mode != "off";
    result.signed_distance = signed_distance;
    result.max_distance = static_cast<float>(max_distance);
    if (unknown_mode != "all") {
      result.unknown_z_min = static_cast<float>(ground_height);
      result.unknown_z_max = static_cast<float>(roi_z_max);
    }
    return result;
  }
};

EsdfParameters readEsdf(const rclcpp::Node::SharedPtr &node,
                        const std::string &prefix, EsdfParameters values) {
  values.unknown_mode = declare_or_get_parameter(
      node, prefix + "esdf_unknown_mode", values.unknown_mode);
  if (values.unknown_mode != "off" && values.unknown_mode != "task_roi" &&
      values.unknown_mode != "all") {
    RCLCPP_WARN(node->get_logger(),
                "Invalid %sesdf_unknown_mode '%s'; using task_roi",
                prefix.c_str(), values.unknown_mode.c_str());
    values.unknown_mode = "task_roi";
  }
  values.roi_z_max = declare_or_get_parameter(node, prefix + "esdf_roi_z_max",
                                              values.roi_z_max);
  values.max_distance = declare_or_get_parameter(
      node, prefix + "esdf_max_distance", values.max_distance);
  values.signed_distance = declare_or_get_parameter(
      node, prefix + "esdf_signed_distance", values.signed_distance);
  return values;
}

}  // namespace

void GridMap::initMap(rclcpp::Node::SharedPtr node) {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  node_ = std::move(node);
  if (!node_) {
    cloud_sub_.reset();
    return;
  }
  loadParameters();
  const bool cloud_reliable = declare_or_get_parameter(
      node_, "grid_map.cloud_reliable", true);
  auto cloud_qos = rclcpp::QoS(10);
  if (cloud_reliable) {
    cloud_qos.reliable();
  } else {
    cloud_qos.best_effort();
  }
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        pointCloudCallback(cloud);
      });
  RCLCPP_INFO(node_->get_logger(), "GridMap subscribed to PointCloud2: %s",
              cloud_sub_->get_topic_name());
#ifdef PLAN_ENV_HAS_BIMAX_MSGS
  if (height_map_enable_) {
    height_map_pub_ = node_->create_publisher<bimax_msgs::msg::HeightMap>(
        height_map_topic_, rclcpp::QoS(10));
  }
#endif
}

void GridMap::loadParameters() {
  const auto parameter = [this](const char *name, auto &value) {
    value = declare_or_get_parameter(node_, name, value);
  };

  // XY dimensions come from the task polygon.
  const double high_resolution = declare_or_get_parameter<double>(
      node_, "grid_map.high_resolution", resolution_);
  const double low_resolution = declare_or_get_parameter<double>(
      node_, "grid_map.low_resolution", low_resolution_);
  const double map_size_z = declare_or_get_parameter<double>(
      node_, "grid_map.map_size_z", map_size_z_);
  const double expansion_margin = declare_or_get_parameter<double>(
      node_, "grid_map.expansion_margin", expansion_margin_);
  if (!configureDualResolution(static_cast<float>(high_resolution),
                               static_cast<float>(low_resolution),
                               static_cast<float>(map_size_z),
                               static_cast<float>(expansion_margin))) {
    RCLCPP_WARN(node_->get_logger(),
                "GridMap: invalid geometry parameters; keeping defaults "
                "(resolution=%.3f, map_size_z=%.3f)",
                resolution_, map_size_z_);
  }

  // Each layer has independent policies; low-layer overrides inherit high.
  const auto occupancy =
      readOccupancy(node_, "grid_map.", OccupancyProbabilities{});
  if (!configureOccupancy(occupancy.config())) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid occupancy parameters; using defaults");
    configureOccupancy(occupancy::LogOddsConfig{});
  }
  const auto low_occupancy = readOccupancy(node_, "grid_map.low_", occupancy);
  if (!configureLowOccupancy(low_occupancy.config())) {
    configureLowOccupancy(occupancy_model_.config());
  }
  const double ground_height =
      declare_or_get_parameter<double>(node_, "grid_map.ground_height", 0.0);
  const auto esdf = readEsdf(node_, "grid_map.", EsdfParameters{});
  if (!configureEsdf(esdf.config(ground_height))) {
    RCLCPP_WARN(node_->get_logger(), "Invalid ESDF parameters; using defaults");
    configureEsdf(esdf::EsdfConfig{});
  }
  const auto low_esdf = readEsdf(node_, "grid_map.low_", esdf);
  if (!configureLowEsdf(low_esdf.config(ground_height))) {
    configureLowEsdf(esdf_config_);
  }
  int esdf_change_voxels = static_cast<int>(esdf_recompute_change_voxels_);
  parameter("grid_map.esdf_recompute_change_voxels", esdf_change_voxels);
  esdf_recompute_change_voxels_ =
      static_cast<std::size_t>(std::max(0, esdf_change_voxels));

  parameter("grid_map.cloud_min_map_z", cloud_min_map_z_);
  parameter("grid_map.cloud_low_z_clear_rays", cloud_low_z_clear_rays_);
  parameter("grid_map.support_filter.enabled", support_filter_enabled_);
  parameter("grid_map.support_filter.ground_z", support_filter_ground_z_);
  parameter("grid_map.support_filter.ground_band", support_filter_ground_band_);
  parameter("grid_map.cloud_min_range", cloud_min_range_);
  parameter("grid_map.local_update_range_x", local_update_range_.x());
  parameter("grid_map.local_update_range_y", local_update_range_.y());
  parameter("grid_map.local_update_range_z", local_update_range_.z());
  if (!std::isfinite(cloud_min_map_z_)) {
    cloud_min_map_z_ = 0.02;
  }
  if (!std::isfinite(cloud_min_range_) || cloud_min_range_ < 0.0) {
    cloud_min_range_ = 0.2;
  }
  if (!std::isfinite(support_filter_ground_z_)) {
    support_filter_ground_z_ = 0.0f;
  }
  if (!std::isfinite(support_filter_ground_band_) ||
      support_filter_ground_band_ < 0.0f) {
    support_filter_ground_band_ = 0.02f;
  }
  if (!local_update_range_.allFinite() ||
      (local_update_range_.array() <= 0.0f).any()) {
    local_update_range_ = Eigen::Vector3f(5.0f, 5.0f, 3.0f);
  }
  parameter("grid_map.motion_frame", motion_frame_);
  parameter("grid_map.frame_id", map_frame_);
  parameter("grid_map.cloud_topic", cloud_topic_);
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
  bool low_tsdf_enabled = low_tsdf_enabled_;
  float low_tsdf_truncation_distance = low_tsdf_truncation_distance_;
  parameter("grid_map.low_tsdf.enabled", low_tsdf_enabled);
  parameter("grid_map.low_tsdf.truncation_distance",
            low_tsdf_truncation_distance);
  if (!configureLowTsdf(low_tsdf_enabled, low_tsdf_truncation_distance)) {
    low_tsdf_enabled_ = false;
    low_tsdf_truncation_distance_ = 0.05f;
  }
  float cloud_leaf_size = static_cast<float>(
      declare_or_get_parameter<double>(node_, "grid_map.point_cloud_leaf_size",
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
}

}  // namespace plan_env
