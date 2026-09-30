#include "plan_env/tsdf.hpp"

#include <algorithm>
#include <cmath>

namespace tsdf {

bool TsdfVolume::configure(const GridGeometry &geometry,
                           const float truncation_distance) {
  if (!std::isfinite(geometry.resolution) || geometry.resolution <= 0.0f ||
      geometry.size_x <= 0 || geometry.size_y <= 0 || geometry.size_z <= 0 ||
      !std::isfinite(truncation_distance) || truncation_distance <= 0.0f) {
    return false;
  }
  geometry_ = geometry;
  truncation_distance_ = truncation_distance;
  const std::size_t voxel_count = static_cast<std::size_t>(geometry.size_x) *
                                  static_cast<std::size_t>(geometry.size_y) *
                                  static_cast<std::size_t>(geometry.size_z);
  voxels_.assign(voxel_count, TsdfVoxel{});
  return true;
}

void TsdfVolume::clear() {
  geometry_ = GridGeometry{};
  voxels_.clear();
}

bool TsdfVolume::valid() const {
  return geometry_.resolution > 0.0f && !voxels_.empty();
}

const GridGeometry &TsdfVolume::geometry() const { return geometry_; }

const std::vector<TsdfVoxel> &TsdfVolume::voxels() const { return voxels_; }

std::vector<TsdfVoxel> &TsdfVolume::voxels() { return voxels_; }

void TsdfVolume::integrateVoxel(const Eigen::Vector3i &voxel_id,
                                const Eigen::Vector3f &ray_origin,
                                const Eigen::Vector3f &ray_end) {
  if (!validVoxel(voxel_id) || !ray_origin.allFinite() || !ray_end.allFinite()) {
    return;
  }
  TsdfVoxel &voxel = voxels_[static_cast<std::size_t>(toAddress(voxel_id))];
  const Eigen::Vector3f center =
      geometry_.origin +
      (voxel_id.cast<float>() + Eigen::Vector3f::Constant(0.5f)) *
          geometry_.resolution;
  const Eigen::Vector3f direction = ray_end - ray_origin;
  const float norm = direction.norm();
  const float projection =
      norm > 0.0f ? (center - ray_origin).dot(direction) / norm : 0.0f;
  const float signed_distance = norm - projection;
  const float distance = std::max(-truncation_distance_,
                                  std::min(truncation_distance_, signed_distance)) /
                         truncation_distance_;
  const float new_weight = voxel.weight + 1.0f;
  voxel.distance = (voxel.distance * voxel.weight + distance) / new_weight;
  voxel.weight = new_weight;
}

int TsdfVolume::toAddress(const Eigen::Vector3i &voxel_id) const {
  return voxel_id.x() * geometry_.size_y * geometry_.size_z +
         voxel_id.y() * geometry_.size_z + voxel_id.z();
}

bool TsdfVolume::validVoxel(const Eigen::Vector3i &voxel_id) const {
  return voxel_id.x() >= 0 && voxel_id.x() < geometry_.size_x &&
         voxel_id.y() >= 0 && voxel_id.y() < geometry_.size_y &&
         voxel_id.z() >= 0 && voxel_id.z() < geometry_.size_z;
}

} // namespace tsdf
