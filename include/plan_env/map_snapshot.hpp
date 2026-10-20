#pragma once

#include "plan_env/esdf.hpp"
#include "plan_env/tsdf.hpp"

#include <cstdint>
#include <vector>

namespace plan_env {

/** Mapping gate driven by consecutive timestamped TF poses. */
enum class MotionState {
  kWaitingForPose,
  kMoving,
  kStationaryCollecting,
  kReadyToMove,
};

/** The last map layer that was committed to the global map. */
enum class FusionMode {
  kSamePosition,
  kDifferentPosition,
};

struct MappingStatus {
  MotionState motion_state = MotionState::kWaitingForPose;
  FusionMode last_fusion_mode = FusionMode::kSamePosition;
  // Number of successfully fused frames in the current local map.
  uint32_t stationary_frame_count = 0;
  // Local-map commit quota.
  uint32_t stationary_frames_required = 30;
  uint64_t position_index = 0;
  uint64_t same_position_fused_frames = 0;
  uint64_t different_position_fused_frames = 0;
  // True only after the current local map has been recalibrated and committed
  // to the global map.
  bool ready_to_move = false;
};

struct EsdfSnapshot {
  // Compatibility fields: these describe the high-resolution/original map.
  GridGeometry geometry;
  std::vector<TsdfVoxel> tsdf_voxels;
  std::vector<OccupancyVoxel> occupancy_voxels;
  std::vector<float> occupancy_probabilities;
  std::vector<OccupancyState> occupancy_states;
  std::vector<EsdfVoxel> esdf_voxels;
  // False when a small occupancy change intentionally reused the previous
  // high-resolution ESDF. Occupancy and low-resolution layers are current.
  bool esdf_current = true;
  Eigen::Vector3f camera_position = Eigen::Vector3f::Zero();
  // Pose used by the motion gate (normally base_footprint), in map frame.
  Eigen::Vector3f motion_position = Eigen::Vector3f::Zero();
  bool tsdf_enabled = false;
  // The expanded low-resolution map is available independently for queries
  // outside the original high-resolution task window.
  GridGeometry expanded_geometry;
  std::vector<TsdfVoxel> expanded_tsdf_voxels;
  std::vector<OccupancyVoxel> expanded_occupancy_voxels;
  std::vector<float> expanded_occupancy_probabilities;
  std::vector<OccupancyState> expanded_occupancy_states;
  std::vector<EsdfVoxel> expanded_esdf_voxels;
  bool expanded_tsdf_enabled = false;
  std::vector<Eigen::Vector2f> original_roi_polygon;
  // The task polygon before the rectangular map window is derived.  Keeping
  // this separately lets observation planning score the original ROI rather
  // than the expanded mapping window.
  std::vector<Eigen::Vector2f> source_roi_polygon;
  std::vector<Eigen::Vector2f> task_roi_polygon;
  bool task_mapping_active = false;
  MappingStatus mapping_status;
  uint64_t version = 0;
};

struct TaskRegion {
  bool valid = false;
  Eigen::Vector2f min = Eigen::Vector2f::Zero();
  Eigen::Vector2f max = Eigen::Vector2f::Zero();
  Eigen::Vector2f original_min = Eigen::Vector2f::Zero();
  Eigen::Vector2f original_max = Eigen::Vector2f::Zero();
  float z_min = 0.0f;
  float z_max = 0.0f;
  float expansion_margin = 0.0f;
  std::vector<Eigen::Vector2d> source_polygon;
  std::vector<Eigen::Vector2f> original_polygon;
  std::vector<Eigen::Vector2f> expanded_polygon;

  bool contains(float x, float y, float z) const {
    return valid && x >= min.x() && x <= max.x() && y >= min.y() &&
           y <= max.y() && z >= z_min && z <= z_max;
  }

  bool containsOriginal(float x, float y, float z) const {
    return valid && x >= original_min.x() && x <= original_max.x() &&
           y >= original_min.y() && y <= original_max.y() && z >= z_min &&
           z <= z_max;
  }
};

}  // namespace plan_env
