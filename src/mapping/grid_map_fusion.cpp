#include "plan_env/grid_map.hpp"

#include "plan_env/ray_integration.hpp"
#include "plan_env/ray_casting.hpp"

#include <algorithm>

namespace plan_env {

bool GridMap::fuseSamePositionFrame() {
  // Temporal fusion stays in the current-position local map.
  if (!accumulateStationRays()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    ++same_position_fused_frames_;
  }
  return true;
}

bool GridMap::fuseDifferentPositionFrame() {
  // A complete local map is recalibrated once against the committed global
  // observations, then replayed into the global voxel volumes.
  const bool cross_position = local_map_cross_position_;
  if (!commitLocalMap()) {
    return false;
  }
  if (cross_position) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    ++different_position_fused_frames_;
  }
  return true;
}

bool GridMap::accumulateStationRays() {
  if (grid_geometry_.resolution <= 0.0f || occupancy_voxels_.empty()) {
    return false;
  }
  std::vector<RayObservation> frame_observations;
  frame_observations.reserve(cloud_points_.size());
  const Eigen::Vector3f ray_origin = camera_position_;
  for (const auto &point : cloud_points_) {
    const bool low_z = point.z() < cloud_min_map_z_;
    if (low_z && !cloud_low_z_clear_rays_) {
      continue;
    }
    frame_observations.push_back(RayObservation{ray_origin, point, low_z});
  }
  filterUnsupportedInteriorObservations(
      &frame_observations, grid_geometry_,
      SupportFilterConfig{support_filter_enabled_, support_filter_ground_z_,
                          support_filter_ground_band_});
  const Eigen::Vector3i size(grid_geometry_.size_x, grid_geometry_.size_y,
                             grid_geometry_.size_z);
  const bool has_ray =
      std::any_of(frame_observations.begin(), frame_observations.end(),
                  [&](const RayObservation &ray) {
                    return ray_casting::intersectsGrid(
                        grid_geometry_.origin, grid_geometry_.resolution, size,
                        ray.origin, ray.endpoint);
                  });
  if (!has_ray) {
    return false;
  }
  // Retain sensor evidence until commit. Local voxel volumes were never read;
  // replaying them here doubled traversal and allocated an unused dense map.
  local_ray_observations_.insert(local_ray_observations_.end(),
                                 frame_observations.begin(),
                                 frame_observations.end());
  if (!registration_backend_.config().enabled) {
    return true;
  }
  for (const auto &observation : frame_observations) {
    if (!observation.low_z) {
      local_surface_points_.push_back(observation.endpoint);
    }
  }
  constexpr std::size_t kMaxLocalSurfacePoints = 100000;
  if (local_surface_points_.size() > kMaxLocalSurfacePoints) {
    const std::size_t excess =
        local_surface_points_.size() - kMaxLocalSurfacePoints;
    local_surface_points_.erase(local_surface_points_.begin(),
                                local_surface_points_.begin() +
                                    static_cast<std::ptrdiff_t>(excess));
  }
  return true;
}

void GridMap::clearLocalMap() {
  local_ray_observations_.clear();
  local_surface_points_.clear();
}

bool GridMap::commitLocalMap() {
  if (local_ray_observations_.empty() || occupancy_voxels_.empty()) {
    return false;
  }

  Eigen::Isometry3f recalibration = Eigen::Isometry3f::Identity();
  bool has_recalibration = false;
  const auto &registration_config = registration_backend_.config();
  if (local_map_cross_position_ && global_map_initialized_ &&
      registration_config.enabled &&
      registration_target_points_.size() >=
          registration_config.min_target_points &&
      local_surface_points_.size() >= registration_config.min_target_points) {
    const registration::Result result = registration_backend_.align(
        local_surface_points_, registration_target_points_,
        Eigen::Isometry3f::Identity());
    if (!result.accepted) {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      last_fusion_mode_ = FusionMode::kDifferentPosition;
      stationary_frame_count_ = 0;
      motion_state_ = MotionState::kStationaryCollecting;
      clearLocalMap();
      return false;
    }
    recalibration = result.pose;
    has_recalibration = true;
  }

  std::vector<RayObservation> corrected_observations;
  if (has_recalibration) {
    corrected_observations.reserve(local_ray_observations_.size());
    for (const auto &observation : local_ray_observations_) {
      RayObservation corrected = observation;
      corrected.origin = recalibration * observation.origin;
      corrected.endpoint = recalibration * observation.endpoint;
      corrected_observations.push_back(corrected);
    }
  }
  if (!integrateRayObservations(
          has_recalibration ? corrected_observations : local_ray_observations_,
          &occupancy_voxels_, tsdf_enabled_ ? &tsdf_volume_ : nullptr,
          grid_geometry_, occupancy_model_)) {
    return false;
  }

  for (const auto &point : local_surface_points_) {
    registration_target_points_.push_back(recalibration * point);
  }
  constexpr std::size_t kMaxRegistrationTargetPoints = 100000;
  if (registration_target_points_.size() > kMaxRegistrationTargetPoints) {
    const std::size_t excess =
        registration_target_points_.size() - kMaxRegistrationTargetPoints;
    registration_target_points_.erase(registration_target_points_.begin(),
                                      registration_target_points_.begin() +
                                          static_cast<std::ptrdiff_t>(excess));
  }

  camera_position_ = recalibration * camera_position_;
  global_map_initialized_ = true;
  global_map_ready_for_current_position_ = true;
  global_map_dirty_ = true;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    last_fusion_mode_ = local_map_cross_position_
                            ? FusionMode::kDifferentPosition
                            : FusionMode::kSamePosition;
  }
  clearLocalMap();
  local_map_cross_position_ = false;
  return true;
}

bool GridMap::updateHighResolutionEsdf() {
  std::size_t changed_voxels = occupancy_voxels_.size();
  if (esdf_initialized_ &&
      last_esdf_occupancy_states_.size() == occupancy_voxels_.size()) {
    changed_voxels = 0;
    for (std::size_t i = 0; i < occupancy_voxels_.size(); ++i) {
      if (occupancy_model_.state(occupancy_voxels_[i]) !=
          last_esdf_occupancy_states_[i]) {
        if (++changed_voxels >= esdf_recompute_change_voxels_) {
          break;
        }
      }
    }
  }
  const bool recompute_esdf = !esdf_initialized_ ||
                              esdf_recompute_change_voxels_ == 0 ||
                              changed_voxels >= esdf_recompute_change_voxels_;
  esdf_current_ = recompute_esdf || changed_voxels == 0;
  if (recompute_esdf) {
    if (!esdf_volume_.computeIfChanged(occupancy_voxels_, occupancy_model_)) {
      if (node_) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "GridMap failed to compute ESDF after global map fusion");
      }
      esdf_current_ = false;
      global_map_ready_for_current_position_ = false;
      return false;
    }
    last_esdf_occupancy_states_.resize(occupancy_voxels_.size());
    for (std::size_t i = 0; i < occupancy_voxels_.size(); ++i) {
      last_esdf_occupancy_states_[i] =
          occupancy_model_.state(occupancy_voxels_[i]);
    }
    esdf_initialized_ = true;
  } else if (node_) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "Reusing high-resolution ESDF: changed_voxels=%zu "
                 "threshold=%zu",
                 changed_voxels, esdf_recompute_change_voxels_);
  }
  return true;
}

}  // namespace plan_env
