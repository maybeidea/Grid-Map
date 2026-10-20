#include "plan_env/grid_map.hpp"
#include "utils/snapshot_query.hpp"

#include <algorithm>
#include <cmath>

namespace plan_env {

using namespace detail;

void GridMap::posToIndex(const Eigen::Vector3d &pos,
                         Eigen::Vector3i &id) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  if (grid_geometry_.resolution <= 0.0f) {
    id.setConstant(INVALID_IDX);
    return;
  }
  id = ((pos - grid_geometry_.origin.cast<double>()) /
        static_cast<double>(grid_geometry_.resolution))
           .array()
           .floor()
           .cast<int>();
}

bool GridMap::isInMap(const Eigen::Vector3i &id) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  return id.x() >= 0 && id.y() >= 0 && id.z() >= 0 &&
         id.x() < grid_geometry_.size_x && id.y() < grid_geometry_.size_y &&
         id.z() < grid_geometry_.size_z;
}

bool GridMap::isInMap(const Eigen::Vector2i &id) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  return id.x() >= 0 && id.y() >= 0 && id.x() < grid_geometry_.size_x &&
         id.y() < grid_geometry_.size_y;
}

bool GridMap::isInMap(const Eigen::Vector3d &pos) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return isInMap(id);
}

bool GridMap::isInMap(const Eigen::Vector2d &pos) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  const double r = getResolution();
  return r > 0.0 && pos.x() >= grid_geometry_.origin.x() &&
         pos.y() >= grid_geometry_.origin.y() &&
         pos.x() < grid_geometry_.origin.x() + grid_geometry_.size_x * r &&
         pos.y() < grid_geometry_.origin.y() + grid_geometry_.size_y * r;
}

Eigen::Vector2i GridMap::pos2dToIndex(const Eigen::Vector2d &pos) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  if (getResolution() <= 0.0) {
    return Eigen::Vector2i::Constant(INVALID_IDX);
  }
  return ((pos - grid_geometry_.origin.head<2>().cast<double>()) /
          getResolution())
      .array()
      .floor()
      .cast<int>();
}

void GridMap::indexToPos(const Eigen::Vector3i &id,
                         Eigen::Vector3d &pos) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  pos = grid_geometry_.origin.cast<double>() +
        (id.cast<double>().array() + 0.5).matrix() * getResolution();
}

Eigen::Vector2d GridMap::index2dToPos(const Eigen::Vector2i &id) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  return grid_geometry_.origin.head<2>().cast<double>() +
         (id.cast<double>().array() + 0.5).matrix() * getResolution();
}

double GridMap::getResolution() const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  return static_cast<double>(resolution_);
}

Eigen::Vector3d GridMap::getOrigin() const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  return grid_geometry_.origin.cast<double>();
}

void GridMap::getVoxelNum(Eigen::Vector3i &voxel_num) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  voxel_num = Eigen::Vector3i(grid_geometry_.size_x, grid_geometry_.size_y,
                              grid_geometry_.size_z);
}

bool GridMap::isMappingReadyForPlanning() const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  const auto status = mappingStatus();
  const auto current = snapshot();
  return task_region_.valid && status.ready_to_move && current &&
         current->task_mapping_active && !current->esdf_voxels.empty() &&
         !current->expanded_esdf_voxels.empty() &&
         current->mapping_status.position_index == status.position_index;
}

void GridMap::resetAstarBuffer() {}

std::shared_ptr<const EsdfSnapshot> GridMap::snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return latest_esdf_snapshot_;
}

std::shared_ptr<const EsdfSnapshot> GridMap::querySnapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return esdf_freezing_query_active_ && frozen_esdf_snapshot_
             ? frozen_esdf_snapshot_
             : latest_esdf_snapshot_;
}

double GridMap::getDistance(const Eigen::Vector3i &id) const {
  const auto selected = querySnapshot();
  return selected ? snapshotDistance(*selected, id) : 100000.0;
}

double GridMap::getDistance(const Eigen::Vector3d &pos) const {
  const auto selected = querySnapshot();
  return selected ? selectedDistance(*selected, pos) : 100000.0;
}

double GridMap::getPreciseDistance(const Eigen::Vector3d &pos) const {
  const auto selected = querySnapshot();
  return selected ? selectedPreciseDistance(*selected, pos) : 100000.0;
}

int GridMap::getOccupancy(const Eigen::Vector3i &id) const {
  const auto selected = querySnapshot();
  return selected ? snapshotOccupancy(*selected, id) : -1;
}

int GridMap::getOccupancy(const Eigen::Vector3d &pos) const {
  const auto selected = querySnapshot();
  return selected ? selectedOccupancy(*selected, pos) : -1;
}

bool GridMap::isUnknown(const Eigen::Vector3i &id) const {
  return getOccupancy(id) < 0;
}

bool GridMap::isUnknown(const Eigen::Vector3d &pos) const {
  return getOccupancy(pos) < 0;
}

bool GridMap::isKnownFree(const Eigen::Vector3i &id) const {
  return getOccupancy(id) == 0;
}

bool GridMap::isKnownOccupied(const Eigen::Vector3i &id) const {
  return getOccupancy(id) == 1;
}

bool GridMap::isKnownOccupied(const Eigen::Vector3d &pos) const {
  return getOccupancy(pos) == 1;
}

void GridMap::captureEsdfFreezing() {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  if (!isMappingReadyForPlanning()) {
    return;
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  frozen_esdf_snapshot_ = latest_esdf_snapshot_;
}

void GridMap::freezeOccupancyAndEsdf() {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  if (!isMappingReadyForPlanning()) {
    return;
  }
  captureEsdfFreezing();
  setEsdfFreezingQueryActive(true);
  occupancy_updates_enabled_ = false;
  setCloudAdmission(false);
}

void GridMap::clearEsdfFreezing() {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  occupancy_updates_enabled_ = true;
  setCloudAdmission(task_region_.valid);
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  frozen_esdf_snapshot_.reset();
  esdf_freezing_query_active_ = false;
}

bool GridMap::hasEsdfFreezing() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return static_cast<bool>(frozen_esdf_snapshot_);
}

void GridMap::setEsdfFreezingQueryActive(const bool active) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  esdf_freezing_query_active_ =
      active && static_cast<bool>(frozen_esdf_snapshot_);
}

bool GridMap::esdfFreezingQueryActive() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return esdf_freezing_query_active_;
}

bool GridMap::odomValid() const {
  return isMappingReadyForPlanning();
}

bool GridMap::getOdomDepthTimeout() const {
  return false;
}

bool GridMap::isInTaskPessimismPolygon(const double x, const double y) const {
  return pointInTaskRegion(static_cast<float>(x), static_cast<float>(y), 0.0f);
}

void GridMap::boundIndex(Eigen::Vector3i &id) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  id.x() = std::max(0, std::min(id.x(), grid_geometry_.size_x - 1));
  id.y() = std::max(0, std::min(id.y(), grid_geometry_.size_y - 1));
  id.z() = std::max(0, std::min(id.z(), grid_geometry_.size_z - 1));
}

void GridMap::boundIndex(Eigen::Vector2i &id) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  id.x() = std::max(0, std::min(id.x(), grid_geometry_.size_x - 1));
  id.y() = std::max(0, std::min(id.y(), grid_geometry_.size_y - 1));
}

void GridMap::getRegion(Eigen::Vector3d &origin, Eigen::Vector3d &size) const {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  origin = grid_geometry_.origin.cast<double>();
  size = Eigen::Vector3d(grid_geometry_.size_x, grid_geometry_.size_y,
                         grid_geometry_.size_z) *
         getResolution();
}

}  // namespace plan_env
