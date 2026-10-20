#include "plan_env/grid_map.hpp"

namespace plan_env {

bool GridMap::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  if (!cloud) {
    return false;
  }

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
  // The low-resolution map is published on every accepted cloud. The
  // high-resolution map is published only after a stationary local map is
  // committed.
  if (global_map_dirty_ || low_map_dirty_) {
    const bool publish_height = global_map_dirty_;
    low_map_dirty_ = false;
    publishEsdfSnapshot();
    global_map_dirty_ = false;
    if (publish_height) {
      publishHeightMap();
    }
    return publish_height;
  }
  // A high-resolution local-only update is intentionally not reported as a
  // planning-ready global-map commit.
  return false;
}

bool GridMap::processPointCloudFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
  if (!task_region_.valid || !occupancy_updates_enabled_) {
    return false;
  }
  PreparedCloud prepared;
  if (!processPointCloud(cloud, &prepared)) {
    return false;
  }

  // The expanded low-resolution map is a realtime layer. It follows the
  // current TF pose and is updated even while the base is moving.
  const bool low_updated = updateLowResolutionMap(prepared);

  if (!updateMotionState(prepared)) {
    // The low layer already updated; the high layer waits for a stable pose.
    if (node_) {
      const auto status = mappingStatus();
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "GridMap did not fuse cloud: motion gate state=%u "
                           "stationary_frames=%u/%u",
                           static_cast<unsigned int>(status.motion_state),
                           status.stationary_frame_count,
                           status.stationary_frames_required);
    }
    return low_updated;
  }

  // The station is accumulated in its odometry/TF pose. Cross-position
  // registration is intentionally delayed until the complete local map is
  // available, immediately before the global commit.
  std::vector<Eigen::Vector3f> transformed_points;
  Eigen::Vector3f high_camera_position = Eigen::Vector3f::Zero();
  prepareMapPoints(prepared, prepared.prior_pose, &transformed_points,
                   &high_camera_position, true);
  if (transformed_points.empty()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap did not fuse cloud: no filtered points remain inside the "
          "active task ROI and map height range");
    }
    return low_updated;
  }
  cloud_points_ = std::move(transformed_points);
  camera_position_ = high_camera_position;
  if (!fuseSamePositionFrame()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "GridMap did not fuse cloud into the local map: ray integration "
          "failed");
    }
    return low_updated;
  }
  recordSuccessfulFusion();

  // The frame quota closes the local map. Only this operation can modify the
  // global occupancy/TSDF volumes and make a new snapshot visible.
  const auto status = mappingStatus();
  if (status.stationary_frame_count >= status.stationary_frames_required) {
    if (!fuseDifferentPositionFrame()) {
      if (node_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "GridMap kept local map: cross-position recalibration rejected "
            "the local map");
      }
      return low_updated;
    }
    if (!updateHighResolutionEsdf()) {
      return low_updated;
    }
  }
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (global_map_ready_for_current_position_) {
      motion_state_ = MotionState::kReadyToMove;
    } else {
      motion_state_ = MotionState::kStationaryCollecting;
    }
  }
  return true;
}

}  // namespace plan_env
