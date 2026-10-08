#include "plan_env/ray_casting.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_env {

namespace ray_casting {
namespace {

bool validGrid(const float resolution, const Eigen::Vector3i &size) {
  return std::isfinite(resolution) && resolution > 0.0f && size.x() > 0 &&
         size.y() > 0 && size.z() > 0;
}

bool clipSegment(const Eigen::Vector3f &min_corner,
                 const Eigen::Vector3f &max_corner,
                 const Eigen::Vector3f &origin, const Eigen::Vector3f &end,
                 float *t_min, float *t_max) {
  float lower = 0.0f;
  float upper = 1.0f;
  const Eigen::Vector3f direction = end - origin;
  constexpr float kParallelEpsilon = 1e-7f;

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) <= kParallelEpsilon) {
      if (origin[axis] < min_corner[axis] || origin[axis] > max_corner[axis]) {
        return false;
      }
      continue;
    }
    float a = (min_corner[axis] - origin[axis]) / direction[axis];
    float b = (max_corner[axis] - origin[axis]) / direction[axis];
    if (a > b) {
      std::swap(a, b);
    }
    lower = std::max(lower, a);
    upper = std::min(upper, b);
    if (lower > upper) {
      return false;
    }
  }
  *t_min = lower;
  *t_max = upper;
  return true;
}

int voxelCoordinate(const float coordinate, const float origin,
                    const float resolution, const int size) {
  const float scaled = (coordinate - origin) / resolution;
  const int index = static_cast<int>(std::floor(scaled));
  return std::max(0, std::min(index, size - 1));
}

}  // namespace

bool pointInGrid(const Eigen::Vector3f &grid_origin, const float resolution,
                 const Eigen::Vector3i &grid_size,
                 const Eigen::Vector3f &point) {
  if (!validGrid(resolution, grid_size) || !point.allFinite()) {
    return false;
  }
  const Eigen::Vector3f max_corner =
      grid_origin + resolution * grid_size.cast<float>();
  return (point.array() >= grid_origin.array()).all() &&
         (point.array() < max_corner.array()).all();
}

bool traverseRay(const Eigen::Vector3f &grid_origin, const float resolution,
                 const Eigen::Vector3i &grid_size,
                 const Eigen::Vector3f &ray_origin,
                 const Eigen::Vector3f &ray_end,
                 const VoxelCallback &callback) {
  if (!callback || !validGrid(resolution, grid_size) ||
      !ray_origin.allFinite() || !ray_end.allFinite()) {
    return false;
  }

  const Eigen::Vector3f max_corner =
      grid_origin + resolution * grid_size.cast<float>();
  float t_min = 0.0f;
  float t_max = 1.0f;
  if (!clipSegment(grid_origin, max_corner, ray_origin, ray_end, &t_min,
                   &t_max)) {
    return false;
  }

  const Eigen::Vector3f direction = ray_end - ray_origin;
  const float length = direction.norm();
  const float epsilon = std::max(1e-6f, resolution * 1e-5f);
  Eigen::Vector3f start = ray_origin + t_min * direction;
  const Eigen::Vector3f finish = ray_origin + t_max * direction;
  if (length > 0.0f) {
    start += direction / length * epsilon;
    if ((start - finish).dot(direction) > 0.0f) {
      start = finish;
    }
  }

  Eigen::Vector3i voxel;
  Eigen::Vector3i last;
  for (int axis = 0; axis < 3; ++axis) {
    voxel[axis] = voxelCoordinate(start[axis], grid_origin[axis], resolution,
                                  grid_size[axis]);
    last[axis] = voxelCoordinate(finish[axis], grid_origin[axis], resolution,
                                 grid_size[axis]);
  }

  const Eigen::Vector3i step_direction(direction.x() > 1e-7f    ? 1
                                       : direction.x() < -1e-7f ? -1
                                                                : 0,
                                       direction.y() > 1e-7f    ? 1
                                       : direction.y() < -1e-7f ? -1
                                                                : 0,
                                       direction.z() > 1e-7f    ? 1
                                       : direction.z() < -1e-7f ? -1
                                                                : 0);
  Eigen::Vector3f t_next;
  Eigen::Vector3f t_delta;
  const float infinity = std::numeric_limits<float>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    if (step_direction[axis] == 0) {
      t_next[axis] = infinity;
      t_delta[axis] = infinity;
      continue;
    }
    const float boundary =
        grid_origin[axis] + (step_direction[axis] > 0
                                 ? (voxel[axis] + 1) * resolution
                                 : voxel[axis] * resolution);
    t_next[axis] = (boundary - start[axis]) / direction[axis];
    t_delta[axis] = resolution / std::abs(direction[axis]);
    if (t_next[axis] < 0.0f) {
      t_next[axis] = 0.0f;
    }
  }

  const bool endpoint_in_grid =
      pointInGrid(grid_origin, resolution, grid_size, ray_end);
  callback(voxel, endpoint_in_grid && voxel == last);
  while (voxel != last) {
    const float next = std::min(t_next.x(), std::min(t_next.y(), t_next.z()));
    if (!std::isfinite(next)) {
      break;
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (t_next[axis] <= next + 1e-6f) {
        voxel[axis] += step_direction[axis];
        t_next[axis] += t_delta[axis];
      }
    }
    if ((voxel.array() < 0).any() ||
        (voxel.array() >= grid_size.array()).any()) {
      break;
    }
    callback(voxel, endpoint_in_grid && voxel == last);
  }
  return true;
}

}  // namespace ray_casting

}  // namespace plan_env
