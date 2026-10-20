#pragma once

#include "plan_env/map_snapshot.hpp"

namespace plan_env {
namespace detail {

double snapshotDistance(const EsdfSnapshot &snapshot,
                        const Eigen::Vector3i &id);
int snapshotOccupancy(const EsdfSnapshot &snapshot, const Eigen::Vector3i &id);
double selectedDistance(const EsdfSnapshot &snapshot,
                        const Eigen::Vector3d &pos);
int selectedOccupancy(const EsdfSnapshot &snapshot, const Eigen::Vector3d &pos);
double selectedPreciseDistance(const EsdfSnapshot &snapshot,
                               const Eigen::Vector3d &pos);

}  // namespace detail
}  // namespace plan_env
