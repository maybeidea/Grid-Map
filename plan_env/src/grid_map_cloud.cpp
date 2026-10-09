#include "plan_env/grid_map.hpp"

#include "plan_env/ray_casting.hpp"

#include <cmath>
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

void GridMap::initMap(rclcpp::Node::SharedPtr node) {
  node_ = std::move(node);
  if (!node_) {
    cloud_sub_.reset();
    return;
  }
  const auto parameter = [this](const char *name, auto &value) {
    node_->get_parameter_or(name, value, value);
  };
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
  float cloud_leaf_size = cloud_leaf_size_;
  parameter("grid_map.point_cloud_leaf_size", cloud_leaf_size);
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
  if (!cloud) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
    if (stopping_) {
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

    if (processPointCloudFrame(cloud)) {
      // ESDF and HeightMap are refreshed once per completed stationary
      // position, rather than once per input cloud.
      publishEsdfSnapshot();
      publishHeightMap();
    }
  }
}

bool GridMap::processPointCloudFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
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

  Eigen::Isometry3f pose = prepared.prior_pose;
  if (mode == FusionMode::kDifferentPosition &&
      !registerDifferentPosition(prepared, &pose)) {
    return false;
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
    return false;
  }
  cloud_points_ = std::move(transformed_points);
  camera_position_ = pose.translation();
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
  const bool fused = mode == FusionMode::kSamePosition
                         ? fuseSamePositionFrame()
                         : fuseDifferentPositionFrame();
  if (!fused || !recordSuccessfulFusion()) {
    return false;
  }

  // The distance field is intentionally recomputed only after the current
  // stationary position has accumulated its successful fusion quota.
  return esdf_volume_.compute(occupancy_voxels_, occupancy_model_);
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
  const Eigen::Matrix3f rotation_matrix =
      map_from_camera.normalized().toRotationMatrix();
  const Eigen::Vector3f translation_vector(static_cast<float>(translation.x),
                                           static_cast<float>(translation.y),
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
    const bool endpoint_in_grid = ray_casting::pointInGrid(
        grid_geometry_.origin, grid_geometry_.resolution, grid_size, point);
    ray_casting::traverseRay(
        grid_geometry_.origin, grid_geometry_.resolution, grid_size, ray_origin,
        point,
        [this, &ray_origin, &point, &integrated_voxel,
         endpoint_in_grid](const Eigen::Vector3i &voxel_id, bool hit) {
          const int address = toAddress(voxel_id);
          if (address < 0 ||
              static_cast<size_t>(address) >= occupancy_voxels_.size()) {
            return;
          }
          integrated_voxel = true;
          occupancy_model_.update(occupancy_voxels_[address], hit);
          if (tsdf_enabled_ && endpoint_in_grid) {
            tsdf_volume_.integrateVoxel(voxel_id, ray_origin, point);
          }
        });
  }
  return integrated_voxel;
}

}  // namespace plan_env
