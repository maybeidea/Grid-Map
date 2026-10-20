#include "utils/snapshot_query.hpp"

#include <cmath>

namespace plan_env {

namespace detail {

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
    return Eigen::Vector3i::Constant(-10000);
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

int snapshotOccupancy(const EsdfSnapshot &snapshot, const Eigen::Vector3i &id);

bool inGeometry(const GridGeometry &geometry, const Eigen::Vector3d &pos,
                Eigen::Vector3i *id = nullptr) {
  const Eigen::Vector3i index = snapshotIndex(geometry, pos);
  if (id) {
    *id = index;
  }
  return snapshotContains(geometry, index);
}

bool useHighResolution(const EsdfSnapshot &snapshot,
                       const Eigen::Vector3d &pos) {
  return inGeometry(snapshot.geometry, pos);
}

const GridGeometry &selectedGeometry(const EsdfSnapshot &snapshot,
                                     const Eigen::Vector3d &pos, bool *high) {
  const bool use_high = useHighResolution(snapshot, pos);
  if (high) {
    *high = use_high;
  }
  return use_high ? snapshot.geometry : snapshot.expanded_geometry;
}

double selectedDistance(const EsdfSnapshot &snapshot,
                        const Eigen::Vector3d &pos) {
  bool high = false;
  const GridGeometry &geometry = selectedGeometry(snapshot, pos, &high);
  const Eigen::Vector3i id = snapshotIndex(geometry, pos);
  if (high) {
    return snapshotDistance(snapshot, id);
  }
  if (!snapshotContains(geometry, id)) {
    return 100000.0;
  }
  const auto address = static_cast<std::size_t>(snapshotAddress(geometry, id));
  return address < snapshot.expanded_esdf_voxels.size()
             ? snapshot.expanded_esdf_voxels[address].distance
             : 100000.0;
}

int selectedOccupancy(const EsdfSnapshot &snapshot,
                      const Eigen::Vector3d &pos) {
  bool high = false;
  const GridGeometry &geometry = selectedGeometry(snapshot, pos, &high);
  const Eigen::Vector3i id = snapshotIndex(geometry, pos);
  if (high) {
    return snapshotOccupancy(snapshot, id);
  }
  if (!snapshotContains(geometry, id)) {
    return -1;
  }
  const auto address = static_cast<std::size_t>(snapshotAddress(geometry, id));
  if (address >= snapshot.expanded_occupancy_states.size()) {
    return -1;
  }
  switch (snapshot.expanded_occupancy_states[address]) {
  case OccupancyState::FREE:
    return 0;
  case OccupancyState::OCCUPIED:
    return 1;
  default:
    return -1;
  }
}

double selectedPreciseDistance(const EsdfSnapshot &snapshot,
                               const Eigen::Vector3d &pos) {
  bool high = false;
  const GridGeometry &geometry = selectedGeometry(snapshot, pos, &high);
  const auto &distances =
      high ? snapshot.esdf_voxels : snapshot.expanded_esdf_voxels;
  if (geometry.resolution <= 0.0f || !pos.allFinite()) {
    return 100000.0;
  }
  const Eigen::Vector3f local =
      (pos.cast<float>() - geometry.origin) / geometry.resolution -
      Eigen::Vector3f::Constant(0.5f);
  const Eigen::Vector3i base = local.array().floor().cast<int>();
  const Eigen::Vector3f frac = local - base.cast<float>();
  auto distance_at = [&](const Eigen::Vector3i &id) {
    if (!snapshotContains(geometry, id)) {
      return 100000.0;
    }
    const auto address =
        static_cast<std::size_t>(snapshotAddress(geometry, id));
    return address < distances.size()
               ? static_cast<double>(distances[address].distance)
               : 100000.0;
  };
  if (base.x() < 0 || base.y() < 0 || base.z() < 0 ||
      base.x() + 1 >= geometry.size_x || base.y() + 1 >= geometry.size_y ||
      base.z() + 1 >= geometry.size_z) {
    return distance_at(snapshotIndex(geometry, pos));
  }
  double value = 0.0;
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        const Eigen::Vector3i id = base + Eigen::Vector3i(dx, dy, dz);
        const float wx = dx ? frac.x() : 1.0f - frac.x();
        const float wy = dy ? frac.y() : 1.0f - frac.y();
        const float wz = dz ? frac.z() : 1.0f - frac.z();
        value += static_cast<double>(wx * wy * wz) * distance_at(id);
      }
    }
  }
  return value;
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

}  // namespace detail

}  // namespace plan_env
