#pragma once

#include "plan_env/map_snapshot.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace plan_env {

/** A base pose selected for one stationary observation. */
struct ObservationTarget {
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  double yaw = 0.0;
  double score = 0.0;
  std::size_t visible_unknown = 0;
  std::size_t visible_wipeable = 0;
};

/** Coverage and reachability evidence used to decide whether to wipe. */
struct ObservationMetrics {
  std::size_t original_cells = 0;
  std::size_t known_cells = 0;
  std::size_t unknown_cells = 0;
  std::size_t wipeable_cells = 0;
  std::size_t opening_cells = 0;
  double unknown_ratio = 1.0;
  bool opening_found = false;
  bool wipeable_region_found = false;
  bool theoretically_unobservable = false;
};

struct ObservationPlan {
  bool valid = false;
  bool complete = false;
  ObservationMetrics metrics;
  ObservationTarget target;
  std::vector<Eigen::Vector2d> wipeable_cells;
  std::string reason;
};

/** Parameters for coverage-first, minimum-view observation planning. */
struct ObservationPlannerConfig {
  // A column is wipeable only when it has free evidence through this band.
  double wipe_z_min = 0.02;
  double wipe_z_max = 0.20;
  double arm_reach = 0.85;
  // Short conservative entry check; this is deliberately not a full arm
  // reachability or swept-volume collision planner.
  double arm_entry_depth = 0.25;
  double opening_width = 0.12;
  double sensor_range = 1.20;
  double sensor_half_fov = 0.65;
  double candidate_spacing = 0.20;
  double candidate_standoff = 0.55;
  double min_wipeable_cells = 8.0;
  // Completion means the remaining unknown fraction in the wipeable ROI is
  // below this value.  An unobservable remainder can be accepted only when
  // no candidate can see it.
  double max_unknown_ratio = 0.18;
  double travel_penalty = 0.08;
  std::size_t max_candidates = 64;
};

/**
 * Planner over one immutable snapshot, retaining only previously wiped cells.
 *
 * The planner deliberately uses the original high-resolution map.  The
 * expanded low-resolution map remains available to Nav2 and collision checks,
 * but cannot make an unknown wipe region look complete.
 */
class ObservationPlanner {
public:
  explicit ObservationPlanner(
      const ObservationPlannerConfig &config = ObservationPlannerConfig{});

  void reset();
  void markWiped(const ObservationPlan &plan);

  bool plan(const EsdfSnapshot &snapshot, const Eigen::Vector2d &current_xy,
            const std::vector<Eigen::Vector2d> &visited_targets,
            ObservationPlan *plan_out) const;

  const ObservationPlannerConfig &config() const { return config_; }

private:
  ObservationPlannerConfig config_;
  std::vector<Eigen::Vector2d> wiped_cells_;
};

}  // namespace plan_env
