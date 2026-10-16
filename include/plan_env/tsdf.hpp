#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace plan_env {

struct GridGeometry {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  float resolution = 0.0f;
  int size_x = 0;
  int size_y = 0;
  int size_z = 0;
};

struct TsdfVoxel {
  float distance = 1.0f;
  float weight = 0.0f;
};

namespace tsdf {

class TsdfVolume {
public:
  TsdfVolume() = default;

  bool configure(const GridGeometry &geometry,
                 float truncation_distance = 0.20f);
  void clear();

  bool valid() const;
  const GridGeometry &geometry() const;
  const std::vector<TsdfVoxel> &voxels() const;
  std::vector<TsdfVoxel> &voxels();

  // Fuse one camera-to-surface ray. Traversal is supplied by the caller so
  // this module owns TSDF state and fusion, while GridMap owns ray policy.
  void integrateVoxel(const Eigen::Vector3i &voxel_id,
                      const Eigen::Vector3f &ray_origin,
                      const Eigen::Vector3f &ray_end);

private:
  int toAddress(const Eigen::Vector3i &voxel_id) const;
  bool validVoxel(const Eigen::Vector3i &voxel_id) const;

  GridGeometry geometry_;
  float truncation_distance_ = 0.20f;
  std::vector<TsdfVoxel> voxels_;
};

}  // namespace tsdf

}  // namespace plan_env
