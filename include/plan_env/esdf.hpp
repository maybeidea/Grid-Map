#pragma once

#include "plan_env/occupancy.hpp"
#include "plan_env/grid_geometry.hpp"

#include <cstddef>
#include <limits>
#include <vector>

namespace plan_env {

struct EsdfVoxel {
  float distance = 100000.0f;
};

namespace esdf {

struct EsdfConfig {
  float max_distance = 100000.0f;
  bool unknown_is_occupied = true;
  bool signed_distance = true;
  // Map-frame height band for unknown sources. Infinite bounds implement all.
  // Bounds select voxel layers like the legacy floor(z / resolution) policy.
  float unknown_z_min = -std::numeric_limits<float>::infinity();
  float unknown_z_max = std::numeric_limits<float>::infinity();
  // Per-volume limit; never changes process-wide OpenMP settings.
  int omp_threads = 2;
  // Exact incremental separable transform; false uses full-volume recompute.
  bool local_update = true;
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

  // With local_update enabled, propagate source changes through affected axis
  // lines; otherwise recompute the volume. Unchanged sources reuse the field.
  // Occupancy probabilities may still change; callers must publish those.
  bool computeIfChanged(const std::vector<OccupancyVoxel> &occupancy,
                        const occupancy::LogOddsModel &model,
                        bool *recomputed = nullptr);

  // Number of 1-D transforms in the last update (both signs combined).
  std::size_t lastUpdatedLines() const { return last_updated_lines_; }

private:
  void updateLocal(const std::vector<int> &changed_columns, bool initialize);
  std::vector<float> positive_z_, positive_y_, negative_z_, negative_y_;
  bool local_cache_valid_ = false;
  std::size_t last_updated_lines_ = 0;
  bool computeImpl(const std::vector<OccupancyVoxel> &occupancy,
                   const occupancy::LogOddsModel &model, bool reuse,
                   bool *recomputed);
  std::vector<float> distance_scratch_;
  std::vector<unsigned char> source_mask_;
  bool sources_valid_ = false;
  GridGeometry geometry_;
  EsdfConfig config_;
  std::vector<EsdfVoxel> voxels_;
};

}  // namespace esdf

}  // namespace plan_env
