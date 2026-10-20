#include "plan_env/ray_integration.hpp"
#include "plan_env/ray_casting.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace plan_env {

void filterUnsupportedInteriorObservations(
    std::vector<RayObservation> *observations, const GridGeometry &geometry,
    const SupportFilterConfig &config) {
  if (!config.enabled || !observations || geometry.resolution <= 0.0f ||
      observations->empty()) {
    return;
  }

  const Eigen::Vector3i size(geometry.size_x, geometry.size_y, geometry.size_z);
  const auto indexOf = [&geometry](const Eigen::Vector3f &point) {
    return ((point - geometry.origin) / geometry.resolution)
        .array()
        .floor()
        .cast<int>()
        .matrix();
  };
  const auto addressOf = [&size](const Eigen::Vector3i &id) {
    return (id.x() * size.y() + id.y()) * size.z() + id.z();
  };
  const auto isBoundary = [&size](const Eigen::Vector3i &id) {
    return id.x() == 0 || id.y() == 0 || id.z() == 0 ||
           id.x() == size.x() - 1 || id.y() == size.y() - 1 ||
           id.z() == size.z() - 1;
  };

  std::unordered_set<int> candidates;
  for (const auto &observation : *observations) {
    if (observation.low_z ||
        !ray_casting::pointInGrid(geometry.origin, geometry.resolution, size,
                                  observation.endpoint)) {
      continue;
    }
    const Eigen::Vector3i id = indexOf(observation.endpoint);
    if (!isBoundary(id)) {
      candidates.insert(addressOf(id));
    }
  }
  if (candidates.empty()) {
    return;
  }

  std::unordered_set<int> supported;
  std::queue<Eigen::Vector3i> queue;
  for (const int address : candidates) {
    const int z = address % size.z();
    const int y = (address / size.z()) % size.y();
    const int x = address / (size.y() * size.z());
    const float center_z = geometry.origin.z() +
                           (static_cast<float>(z) + 0.5f) * geometry.resolution;
    if (std::abs(center_z - config.ground_z) <= config.ground_band) {
      queue.emplace(x, y, z);
      supported.insert(address);
    }
  }
  constexpr int kNeighborOffsets[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  while (!queue.empty()) {
    const Eigen::Vector3i current = queue.front();
    queue.pop();
    for (const auto &offset : kNeighborOffsets) {
      const Eigen::Vector3i next =
          current + Eigen::Vector3i(offset[0], offset[1], offset[2]);
      if (isBoundary(next)) {
        continue;
      }
      const int address = addressOf(next);
      if (candidates.count(address) != 0 && supported.insert(address).second) {
        queue.push(next);
      }
    }
  }

  observations->erase(
      std::remove_if(
          observations->begin(), observations->end(),
          [&](const RayObservation &observation) {
            if (observation.low_z ||
                !ray_casting::pointInGrid(geometry.origin, geometry.resolution,
                                          size, observation.endpoint)) {
              return false;
            }
            const Eigen::Vector3i id = indexOf(observation.endpoint);
            return !isBoundary(id) && supported.count(addressOf(id)) == 0;
          }),
      observations->end());
}

bool integrateRayObservations(const std::vector<RayObservation> &observations,
                              std::vector<OccupancyVoxel> *occupancy,
                              tsdf::TsdfVolume *tsdf,
                              const GridGeometry &geometry,
                              const occupancy::LogOddsModel &model) {
  if (!occupancy || occupancy->empty() || geometry.resolution <= 0.0f) {
    return false;
  }
  const Eigen::Vector3i grid_size(geometry.size_x, geometry.size_y,
                                  geometry.size_z);
  bool integrated_voxel = false;
  for (const auto &observation : observations) {
    const Eigen::Vector3f &ray_origin = observation.origin;
    const Eigen::Vector3f &point = observation.endpoint;
    const bool low_z = observation.low_z;
    const bool endpoint_in_grid = ray_casting::pointInGrid(
        geometry.origin, geometry.resolution, grid_size, point);
    ray_casting::traverseRay(
        geometry.origin, geometry.resolution, grid_size, ray_origin, point,
        [occupancy, tsdf, &geometry, &model, &ray_origin, &point,
         &integrated_voxel, endpoint_in_grid,
         low_z](const Eigen::Vector3i &voxel_id, bool hit) {
          const int address = voxel_id.x() * geometry.size_y * geometry.size_z +
                              voxel_id.y() * geometry.size_z + voxel_id.z();
          if (address < 0 ||
              static_cast<size_t>(address) >= occupancy->size()) {
            return;
          }
          integrated_voxel = true;
          model.update((*occupancy)[address], hit && !low_z);
          if (tsdf && endpoint_in_grid && !low_z) {
            tsdf->integrateVoxel(voxel_id, ray_origin, point);
          }
        });
  }
  return integrated_voxel;
}

}  // namespace plan_env
