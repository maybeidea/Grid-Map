#pragma once

#include "plan_env/occupancy.hpp"
#include "plan_env/tsdf.hpp"

namespace plan_env {

struct RayObservation {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  Eigen::Vector3f endpoint = Eigen::Vector3f::Zero();
  bool low_z = false;
};

struct SupportFilterConfig {
  bool enabled = false;
  float ground_z = 0.0f;
  float ground_band = 0.02f;
};

// Preserve boundary hits and ground clearing rays; retain interior hits only
// when their six-connected component touches the configured ground band.
void filterUnsupportedInteriorObservations(
    std::vector<RayObservation> *observations, const GridGeometry &geometry,
    const SupportFilterConfig &config);

// Walk each observation once, sharing traversal between occupancy and TSDF.
bool integrateRayObservations(const std::vector<RayObservation> &observations,
                              std::vector<OccupancyVoxel> *occupancy,
                              tsdf::TsdfVolume *tsdf,
                              const GridGeometry &geometry,
                              const occupancy::LogOddsModel &model);

}  // namespace plan_env
