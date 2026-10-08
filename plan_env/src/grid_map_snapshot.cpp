#include "plan_env/grid_map.hpp"

#include <cerrno>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/stat.h>

namespace plan_env {

bool GridMap::buildEsdfFreezingSliceCloud(
    sensor_msgs::msg::PointCloud2 &cloud_msg) const {
  std::shared_ptr<const EsdfSnapshot> frozen;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    frozen = frozen_esdf_snapshot_;
  }
  if (!frozen || frozen->geometry.size_x <= 0 || frozen->geometry.size_y <= 0 ||
      frozen->geometry.size_z <= 0) {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const auto &geometry = frozen->geometry;
  for (int x = 0; x < geometry.size_x; ++x) {
    for (int y = 0; y < geometry.size_y; ++y) {
      bool occupied = false;
      bool known = false;
      for (int z = 0; z < geometry.size_z; ++z) {
        const int address =
            x * geometry.size_y * geometry.size_z + y * geometry.size_z + z;
        const auto state =
            frozen->occupancy_states[static_cast<size_t>(address)];
        occupied = occupied || state == OccupancyState::OCCUPIED;
        known = known || state != OccupancyState::UNKNOWN;
      }
      pcl::PointXYZI point;
      point.x = geometry.origin.x() + (x + 0.5f) * geometry.resolution;
      point.y = geometry.origin.y() + (y + 0.5f) * geometry.resolution;
      point.z = geometry.origin.z();
      point.intensity = occupied ? 100.0f : (known ? 0.0f : -1.0f);
      cloud.push_back(point);
    }
  }
  if (cloud.empty()) {
    return false;
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = map_frame_;
  if (node_) {
    cloud_msg.header.stamp = node_->now();
  }
  return true;
}

bool GridMap::buildEsdfFreezingVolumeCloud(
    sensor_msgs::msg::PointCloud2 &cloud_msg) const {
  std::shared_ptr<const EsdfSnapshot> frozen;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    frozen = frozen_esdf_snapshot_;
  }
  if (!frozen) {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const auto &geometry = frozen->geometry;
  for (int x = 0; x < geometry.size_x; ++x) {
    for (int y = 0; y < geometry.size_y; ++y) {
      for (int z = 0; z < geometry.size_z; ++z) {
        const int address =
            x * geometry.size_y * geometry.size_z + y * geometry.size_z + z;
        if (address >= static_cast<int>(frozen->occupancy_states.size()) ||
            frozen->occupancy_states[address] != OccupancyState::OCCUPIED) {
          continue;
        }
        pcl::PointXYZI point;
        point.x = geometry.origin.x() + (x + 0.5f) * geometry.resolution;
        point.y = geometry.origin.y() + (y + 0.5f) * geometry.resolution;
        point.z = geometry.origin.z() + (z + 0.5f) * geometry.resolution;
        point.intensity = 100.0f;
        cloud.push_back(point);
      }
    }
  }
  if (cloud.empty()) {
    return false;
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = map_frame_;
  if (node_) {
    cloud_msg.header.stamp = node_->now();
  }
  return true;
}

bool GridMap::saveEsdfSnapshot(const std::string &output_dir,
                               const std::string &tag,
                               std::string *saved_basename_out,
                               const bool save_local3d,
                               const bool save_slice) const {
  if (output_dir.empty() || tag.empty()) {
    return false;
  }
  if (::mkdir(output_dir.c_str(), 0755) != 0 && errno != EEXIST) {
    return false;
  }
  const std::string base = output_dir + "/" + tag;
  bool saved = false;
  sensor_msgs::msg::PointCloud2 cloud;
  if (save_slice && buildEsdfFreezingSliceCloud(cloud)) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    saved = pcl::io::savePCDFileBinary(base + "_slice.pcd", pcl_cloud) == 0 ||
            saved;
  }
  if (save_local3d && buildEsdfFreezingVolumeCloud(cloud)) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    saved = pcl::io::savePCDFileBinary(base + "_local3d.pcd", pcl_cloud) == 0 ||
            saved;
  }
  if (saved_basename_out) {
    *saved_basename_out = base;
  }
  return saved;
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
    latest_esdf_snapshot_ =
        std::shared_ptr<const EsdfSnapshot>(std::move(next));
  }
}

}  // namespace plan_env
