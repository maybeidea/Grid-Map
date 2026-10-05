#pragma once

#include <Eigen/Core>

#include <functional>

namespace plan_env {

namespace ray_casting {

using VoxelCallback =
    std::function<void(const Eigen::Vector3i &voxel, bool hit)>;

bool pointInGrid(const Eigen::Vector3f &grid_origin, float resolution,
                 const Eigen::Vector3i &grid_size,
                 const Eigen::Vector3f &point);

// Walk a camera-to-point segment once. The callback receives each traversed
// voxel in order; hit is true only for the endpoint voxel when the endpoint is
// inside the map. No ray or voxel list is retained by this module.
bool traverseRay(const Eigen::Vector3f &grid_origin, float resolution,
                 const Eigen::Vector3i &grid_size,
                 const Eigen::Vector3f &ray_origin,
                 const Eigen::Vector3f &ray_end,
                 const VoxelCallback &callback);

} // namespace ray_casting

}  // namespace plan_env
