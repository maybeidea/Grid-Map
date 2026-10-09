#include "plan_env/grid_map.hpp"

#include <cmath>

namespace plan_env {

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

bool GridMap::updateMotionState(const PreparedCloud &prepared,
                                FusionMode *mode) {
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

  // Stop accepting frames once this stop reaches its configured quota.
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

  // The quota is advanced only after the map integrator confirms that this
  // cloud was actually fused. This function only admits the candidate frame.
  motion_state_ = MotionState::kStationaryCollecting;
  last_fusion_mode_ = *mode;
  return true;
}

bool GridMap::recordSuccessfulFusion() {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  ++stationary_frame_count_;
  if (stationary_frame_count_ >= stationary_frames_required_) {
    stationary_frame_count_ = stationary_frames_required_;
    motion_state_ = MotionState::kReadyToMove;
    return true;
  }
  motion_state_ = MotionState::kStationaryCollecting;
  return false;
}

}  // namespace plan_env
