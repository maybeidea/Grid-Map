#include "plan_env/grid_map.hpp"
#include "plan_env/height_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#if __has_include(<tf2/exceptions.hpp>)
#include <tf2/exceptions.hpp>
#else
#include <tf2/exceptions.h>
#endif
#if __has_include(<tf2/time.hpp>)
#include <tf2/time.hpp>
#else
#include <tf2/time.h>
#endif

#ifdef PLAN_ENV_HAS_BIMAX_MSGS

namespace plan_env {

namespace {

void zBounds(const GridMap &map, const double z_max, int *z0, int *z1) {
  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  map.getRegion(origin, size);
  const double resolution = map.getResolution();
  Eigen::Vector3i low;
  Eigen::Vector3i high;
  map.posToIndex(
      origin + Eigen::Vector3d(0.5 * resolution, 0.5 * resolution, 0.0), low);
  map.posToIndex(origin +
                     Eigen::Vector3d(0.5 * resolution, 0.5 * resolution, z_max),
                 high);
  map.boundIndex(low);
  map.boundIndex(high);
  *z0 = std::min(low.z(), high.z());
  *z1 = std::max(low.z(), high.z());
}

}  // namespace

bool GridMap::buildHeightMapMsg(bimax_msgs::msg::HeightMap &msg,
                                const Eigen::Isometry3d &T_map_from_height) {
  if (!isTaskMappingActive() || grid_geometry_.size_x <= 0 ||
      grid_geometry_.size_y <= 0 || occupancy_voxels_.empty()) {
    return false;
  }

  const int width = grid_geometry_.size_x;
  const int height = grid_geometry_.size_y;
  const std::size_t count = static_cast<std::size_t>(width) * height;
  const double resolution = grid_geometry_.resolution;
  const HeightBandParams band{0.0, height_floor_tol_, height_z_tcp_min_, 0.10,
                              height_ceil_threshold_open_};

  int z0_status = 0;
  int z1_status = 0;
  int z0_complete = 0;
  int z1_complete = 0;
  zBounds(*this, height_band_z_max_, &z0_status, &z1_status);
  zBounds(*this, height_completeness_z_max_, &z0_complete, &z1_complete);

  msg.header.frame_id = height_map_frame_;
  msg.info.resolution = static_cast<float>(resolution);
  msg.info.width = static_cast<std::uint32_t>(width);
  msg.info.height = static_cast<std::uint32_t>(height);
  msg.info.origin.position.x = grid_geometry_.origin.x();
  msg.info.origin.position.y = grid_geometry_.origin.y();
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.h_floor.assign(count, static_cast<float>(band.ground_height));
  msg.h_ceil.assign(count, std::numeric_limits<float>::infinity());
  msg.status.assign(count, HeightStatus::kUnknown);
  msg.observation_completeness.assign(count, 0.0f);
  msg.h_obj_low.clear();

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const Eigen::Vector3d height_point(
          grid_geometry_.origin.x() + (col + 0.5) * resolution,
          grid_geometry_.origin.y() + (row + 0.5) * resolution, 0.0);
      const Eigen::Vector3d map_point = T_map_from_height * height_point;
      Eigen::Vector3i id;
      posToIndex(map_point, id);
      const std::size_t flat = static_cast<std::size_t>(row) * width + col;
      if (!isInMap(id)) {
        continue;
      }

      int known = 0;
      for (int z = z0_complete; z <= z1_complete; ++z) {
        const OccupancyState state =
            occupancy_model_.state(occupancy_voxels_[static_cast<std::size_t>(
                toAddress(id.x(), id.y(), z))]);
        if (state == OccupancyState::FREE ||
            state == OccupancyState::OCCUPIED) {
          ++known;
        }
      }
      msg.observation_completeness[flat] =
          columnKnownFraction(known, z1_complete - z0_complete + 1);

      bool occupied = false;
      bool free = false;
      double lowest = std::numeric_limits<double>::infinity();
      for (int z = z0_status; z <= z1_status; ++z) {
        const OccupancyState state =
            occupancy_model_.state(occupancy_voxels_[static_cast<std::size_t>(
                toAddress(id.x(), id.y(), z))]);
        if (state == OccupancyState::OCCUPIED) {
          const double relative_z = (z + 0.5) * resolution +
                                    grid_geometry_.origin.z() -
                                    band.ground_height;
          if (occupiedCountsForHeight(relative_z, band)) {
            occupied = true;
            lowest = std::min(lowest, relative_z);
          }
        } else if (state == OccupancyState::FREE) {
          free = true;
        }
      }
      msg.status[flat] = classifyHeightColumn(occupied, free, lowest, band);
      if (occupied && (msg.status[flat] == HeightStatus::kBaseOpen ||
                       (msg.status[flat] == HeightStatus::kAllOpen &&
                        lowest >= band.ceil_threshold_open))) {
        msg.h_ceil[flat] = static_cast<float>(lowest);
      }
    }
  }
  return true;
}

bool GridMap::buildHeightMapMsg(bimax_msgs::msg::HeightMap &msg) {
  return buildHeightMapMsg(msg, Eigen::Isometry3d::Identity());
}

void GridMap::publishHeightMap() {
  if (!height_map_enable_ || !height_map_pub_ ||
      height_map_pub_->get_subscription_count() == 0 || !node_) {
    return;
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (height_map_frame_ != map_frame_ && tf_buffer_) {
    try {
      const auto tf_msg = tf_buffer_->lookupTransform(
          map_frame_, height_map_frame_, tf2::TimePointZero);
      const auto &t = tf_msg.transform.translation;
      const auto &q = tf_msg.transform.rotation;
      Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
      rotation.normalize();
      transform.linear() = rotation.toRotationMatrix();
      transform.translation() = Eigen::Vector3d(t.x, t.y, t.z);
    } catch (const tf2::TransformException &) {
      return;
    }
  }
  bimax_msgs::msg::HeightMap msg;
  if (!buildHeightMapMsg(msg, transform)) {
    return;
  }
  msg.header.stamp = node_->now();
  msg.info.map_load_time = msg.header.stamp;
  height_map_pub_->publish(msg);
}

}  // namespace plan_env

#else

namespace plan_env {

bool GridMap::buildHeightMapMsg(bimax_msgs::msg::HeightMap &,
                                const Eigen::Isometry3d &) {
  return false;
}

bool GridMap::buildHeightMapMsg(bimax_msgs::msg::HeightMap &) {
  return false;
}

void GridMap::publishHeightMap() {}

}  // namespace plan_env

#endif
