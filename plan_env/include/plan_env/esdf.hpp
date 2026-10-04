#pragma once

#include "plan_env/occupancy.hpp"
#include "plan_env/tsdf.hpp"

#include <cstddef>
#include <vector>

struct EsdfVoxel {
  float distance = 100000.0f;
};

namespace esdf {

struct EsdfConfig {
  float max_distance = 100000.0f;
  bool unknown_is_occupied = false;
  bool signed_distance = false;
};

class EsdfVolume {
public:
  EsdfVolume() = default;

  bool configure(const GridGeometry &geometry,
                 const EsdfConfig &config = EsdfConfig{});
  void clear();
  bool valid() const;
  const GridGeometry &geometry() const;
  const std::vector<EsdfVoxel> &voxels() const;
  std::vector<EsdfVoxel> &voxels();

  // Compute a distance field from the current occupancy decisions. The
  // three-pass squared distance transform is linear in the number of voxels.
  bool compute(const std::vector<OccupancyVoxel> &occupancy,
               const occupancy::LogOddsModel &model);
  bool compute(const std::vector<OccupancyVoxel> &occupancy);

private:
  GridGeometry geometry_;
  EsdfConfig config_;
  std::vector<EsdfVoxel> voxels_;
};

} // namespace esdf
