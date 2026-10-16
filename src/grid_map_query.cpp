#include "plan_env/grid_map.hpp"

#include <algorithm>
#include <cmath>

namespace plan_env {

namespace {

int snapshotAddress(const GridGeometry &geometry, const Eigen::Vector3i &id) {
  return id.x() * geometry.size_y * geometry.size_z + id.y() * geometry.size_z +
         id.z();
}

bool snapshotContains(const GridGeometry &geometry, const Eigen::Vector3i &id) {
  return (id.array() >= 0).all() && id.x() < geometry.size_x &&
         id.y() < geometry.size_y && id.z() < geometry.size_z;
}

Eigen::Vector3i snapshotIndex(const GridGeometry &geometry,
                              const Eigen::Vector3d &pos) {
  if (geometry.resolution <= 0.0f || !pos.allFinite()) {
    return Eigen::Vector3i::Constant(GridMap::INVALID_IDX);
  }
  return ((pos - geometry.origin.cast<double>()) / geometry.resolution)
      .array()
      .floor()
      .cast<int>();
}

double snapshotDistance(const EsdfSnapshot &snapshot,
                        const Eigen::Vector3i &id) {
  if (!snapshotContains(snapshot.geometry, id)) {
    return 100000.0;
  }
  const auto address =
      static_cast<std::size_t>(snapshotAddress(snapshot.geometry, id));
  return address < snapshot.esdf_voxels.size()
             ? snapshot.esdf_voxels[address].distance
             : 100000.0;
}

int snapshotOccupancy(const EsdfSnapshot &snapshot, const Eigen::Vector3i &id) {
  if (!snapshotContains(snapshot.geometry, id)) {
    return -1;
  }
  const auto address =
      static_cast<std::size_t>(snapshotAddress(snapshot.geometry, id));
  if (address >= snapshot.occupancy_states.size()) {
    return -1;
  }
  switch (snapshot.occupancy_states[address]) {
  case OccupancyState::FREE:
    return 0;
  case OccupancyState::OCCUPIED:
    return 1;
  default:
    return -1;
  }
}

}  // namespace

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
  return selected ? snapshotDistance(*selected,
                                     snapshotIndex(selected->geometry, pos))
                  : 100000.0;
}

double GridMap::getPreciseDistance(const Eigen::Vector3d &pos) const {
  const auto selected = querySnapshot();
  if (!selected || selected->geometry.resolution <= 0.0f || !pos.allFinite()) {
    return 100000.0;
  }
  const Eigen::Vector3f local =
      (pos.cast<float>() - selected->geometry.origin) /
          selected->geometry.resolution -
      Eigen::Vector3f::Constant(0.5f);
  const Eigen::Vector3i base = local.array().floor().cast<int>();
  const Eigen::Vector3f frac = local - base.cast<float>();
  if (base.x() < 0 || base.y() < 0 || base.z() < 0 ||
      base.x() + 1 >= selected->geometry.size_x ||
      base.y() + 1 >= selected->geometry.size_y ||
      base.z() + 1 >= selected->geometry.size_z) {
    return snapshotDistance(*selected, snapshotIndex(selected->geometry, pos));
  }
  double value = 0.0;
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        const Eigen::Vector3i id = base + Eigen::Vector3i(dx, dy, dz);
        const float wx = dx ? frac.x() : 1.0f - frac.x();
        const float wy = dy ? frac.y() : 1.0f - frac.y();
        const float wz = dz ? frac.z() : 1.0f - frac.z();
        value +=
            static_cast<double>(wx * wy * wz) * snapshotDistance(*selected, id);
      }
    }
  }
  return value;
}

int GridMap::getOccupancy(const Eigen::Vector3i &id) const {
  const auto selected = querySnapshot();
  return selected ? snapshotOccupancy(*selected, id) : -1;
}

int GridMap::getOccupancy(const Eigen::Vector3d &pos) const {
  const auto selected = querySnapshot();
  return selected ? snapshotOccupancy(*selected,
                                      snapshotIndex(selected->geometry, pos))
                  : -1;
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
  if (!isMappingReadyForPlanning()) return;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  frozen_esdf_snapshot_ = latest_esdf_snapshot_;
}

void GridMap::freezeOccupancyAndEsdf() {
  std::lock_guard<std::recursive_mutex> state_lock(task_region_mutex_);
  if (!isMappingReadyForPlanning()) return;
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
  esdf_freezing_query_active_ = active && static_cast<bool>(frozen_esdf_snapshot_);
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
