#include "plan_env/observation_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_env {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

bool pointInPolygon(const Eigen::Vector2d &point,
                   const std::vector<Eigen::Vector2f> &polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  std::size_t j = polygon.size() - 1;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const double xi = polygon[i].x();
    const double yi = polygon[i].y();
    const double xj = polygon[j].x();
    const double yj = polygon[j].y();
    const bool crosses = ((yi > point.y()) != (yj > point.y()));
    if (crosses) {
      const double x_at_y = (xj - xi) * (point.y() - yi) /
                                (yj - yi) + xi;
      if (point.x() < x_at_y) inside = !inside;
    }
    j = i;
  }
  return inside;
}

double distanceToSegment(const Eigen::Vector2d &point,
                         const Eigen::Vector2d &a,
                         const Eigen::Vector2d &b,
                         Eigen::Vector2d *closest) {
  const Eigen::Vector2d d = b - a;
  const double denom = d.squaredNorm();
  const double t = denom > kEps
                       ? std::max(0.0, std::min(1.0, (point - a).dot(d) / denom))
                       : 0.0;
  if (closest) *closest = a + t * d;
  return (point - (a + t * d)).norm();
}

bool nearAny(const Eigen::Vector2d &point,
             const std::vector<Eigen::Vector2d> &points, const double radius) {
  const double radius_sq = radius * radius;
  for (const auto & other : points) {
    if ((point - other).squaredNorm() <= radius_sq) return true;
  }
  return false;
}

struct GridView {
  int width = 0;
  int height = 0;
  int layers = 0;
  double resolution = 0.0;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  std::vector<uint8_t> state;  // 0 free, 1 occupied, 2 unknown

  std::size_t index(const int x, const int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
  }

  bool inBounds(const int x, const int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  Eigen::Vector2d position(const int x, const int y) const {
    return Eigen::Vector2d(origin.x() + (x + 0.5) * resolution,
                           origin.y() + (y + 0.5) * resolution);
  }

  bool indexOf(const Eigen::Vector2d &point, int *x, int *y) const {
    if (!x || !y || resolution <= 0.0) return false;
    *x = static_cast<int>(std::floor((point.x() - origin.x()) / resolution));
    *y = static_cast<int>(std::floor((point.y() - origin.y()) / resolution));
    return inBounds(*x, *y);
  }
};

bool buildGrid(const EsdfSnapshot &snapshot, const ObservationPlannerConfig &config,
               const std::vector<Eigen::Vector2f> &polygon, GridView *grid,
               ObservationMetrics *metrics, std::vector<Eigen::Vector2d> *wipeable) {
  if (!grid || !metrics || !wipeable || snapshot.geometry.size_x <= 0 ||
      snapshot.geometry.size_y <= 0 || snapshot.geometry.size_z <= 0 ||
      snapshot.geometry.resolution <= 0.0f || polygon.size() < 3) {
    return false;
  }
  grid->width = snapshot.geometry.size_x;
  grid->height = snapshot.geometry.size_y;
  grid->layers = snapshot.geometry.size_z;
  grid->resolution = snapshot.geometry.resolution;
  grid->origin = snapshot.geometry.origin.cast<double>();
  grid->state.assign(static_cast<std::size_t>(grid->width) * grid->height, 2);
  wipeable->clear();
  metrics->original_cells = 0;
  metrics->known_cells = 0;
  metrics->unknown_cells = 0;
  metrics->wipeable_cells = 0;
  metrics->opening_cells = 0;

  const int z0 = std::max(
      0, static_cast<int>(std::floor((config.wipe_z_min - grid->origin.z()) /
                                      grid->resolution)));
  const int z1 = std::min(
      grid->layers - 1,
      static_cast<int>(std::floor((config.wipe_z_max - grid->origin.z()) /
                                  grid->resolution)));
  for (int y = 0; y < grid->height; ++y) {
    for (int x = 0; x < grid->width; ++x) {
      const Eigen::Vector2d point = grid->position(x, y);
      if (!pointInPolygon(point, polygon)) continue;
      ++metrics->original_cells;
      bool free = false;
      bool occupied = false;
      bool known = false;
      for (int z = std::max(0, z0); z <= z1; ++z) {
        const std::size_t address = static_cast<std::size_t>(
            (x * grid->height + y) * grid->layers + z);
        if (address >= snapshot.occupancy_states.size()) continue;
        const auto state = snapshot.occupancy_states[address];
        free = free || state == OccupancyState::FREE;
        occupied = occupied || state == OccupancyState::OCCUPIED;
        known = known || state != OccupancyState::UNKNOWN;
      }
      uint8_t cell = 2;
      if (occupied) cell = 1;
      else if (free) cell = 0;
      grid->state[grid->index(x, y)] = cell;
      if (cell != 2) ++metrics->known_cells;
      else ++metrics->unknown_cells;
      if (cell == 0) {
        ++metrics->wipeable_cells;
        wipeable->push_back(point);
      }
      (void)known;
    }
  }
  metrics->unknown_ratio = metrics->original_cells == 0
                               ? 1.0
                               : static_cast<double>(metrics->unknown_cells) /
                                     static_cast<double>(metrics->original_cells);
  return metrics->original_cells > 0;
}

bool rayVisible(const GridView &grid, const Eigen::Vector2d &from,
                const Eigen::Vector2d &to, const bool allow_unknown) {
  const double length = (to - from).norm();
  const int steps = std::max(1, static_cast<int>(std::ceil(
      length / std::max(0.5 * grid.resolution, 0.01))));
  bool entered_map = false;
  for (int i = 1; i <= steps; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector2d point = from + alpha * (to - from);
    int x = 0;
    int y = 0;
    if (!grid.indexOf(point, &x, &y)) {
      // A station may be in the expanded low-resolution window while the
      // visibility target is in the original high-resolution window.
      if (!entered_map && i < steps) continue;
      return false;
    }
    entered_map = true;
    const uint8_t state = grid.state[grid.index(x, y)];
    if (state == 1) return false;
    if (!allow_unknown && state == 2 && i < steps) return false;
  }
  return entered_map;
}

bool columnKnownFree(const GridGeometry &geometry,
                     const std::vector<OccupancyState> &states,
                     const Eigen::Vector2d &point, const double z_min,
                     const double z_max) {
  if (geometry.resolution <= 0.0f || geometry.size_x <= 0 ||
      geometry.size_y <= 0 || geometry.size_z <= 0) {
    return false;
  }
  const int x = static_cast<int>(std::floor(
      (point.x() - geometry.origin.x()) / geometry.resolution));
  const int y = static_cast<int>(std::floor(
      (point.y() - geometry.origin.y()) / geometry.resolution));
  if (x < 0 || x >= geometry.size_x || y < 0 || y >= geometry.size_y)
    return false;
  const int z0 = std::max(
      0, static_cast<int>(std::floor((z_min - geometry.origin.z()) /
                                      geometry.resolution)));
  const int z1 = std::min(
      geometry.size_z - 1,
      static_cast<int>(std::floor((z_max - geometry.origin.z()) /
                                  geometry.resolution)));
  if (z1 < z0) return false;
  for (int z = z0; z <= z1; ++z) {
    const std::size_t address = static_cast<std::size_t>(
        (x * geometry.size_y + y) * geometry.size_z + z);
    if (address >= states.size() || states[address] != OccupancyState::FREE)
      return false;
  }
  return true;
}

bool armEntryCorridorFree(const EsdfSnapshot &snapshot,
                          const ObservationPlannerConfig &config,
                          const Eigen::Vector2d &boundary_point,
                          const Eigen::Vector2d &inward) {
  if (config.candidate_standoff > config.arm_reach ||
      config.arm_entry_depth <= 0.0 || config.opening_width <= 0.0) {
    return false;
  }
  const Eigen::Vector2d tangent(-inward.y(), inward.x());
  const double step = std::max(
      0.5 * static_cast<double>(snapshot.expanded_geometry.resolution), 0.01);
  const int longitudinal_samples = std::max(
      1, static_cast<int>(std::ceil(
             (config.candidate_standoff + config.arm_entry_depth) / step)));
  const double half_width = 0.5 * config.opening_width;
  const double lateral_offsets[] = {-half_width, 0.0, half_width};
  for (int i = 0; i <= longitudinal_samples; ++i) {
    const double distance = -config.candidate_standoff +
                            (config.candidate_standoff +
                             config.arm_entry_depth) *
                                static_cast<double>(i) /
                                static_cast<double>(longitudinal_samples);
    for (const double lateral : lateral_offsets) {
      const Eigen::Vector2d sample = boundary_point + inward * distance +
                                     tangent * lateral;
      if (!columnKnownFree(snapshot.expanded_geometry,
                           snapshot.expanded_occupancy_states, sample,
                           config.wipe_z_min, config.wipe_z_max)) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> largestFreeComponent(
    const GridView &grid, const std::vector<Eigen::Vector2d> &cells) {
  std::vector<Eigen::Vector2d> largest;
  if (cells.empty()) return largest;
  std::vector<uint8_t> present(
      static_cast<std::size_t>(grid.width) * grid.height, 0);
  for (const auto &cell : cells) {
    int x = 0;
    int y = 0;
    if (grid.indexOf(cell, &x, &y)) present[grid.index(x, y)] = 1;
  }
  std::vector<uint8_t> visited(present.size(), 0);
  const int neighbors[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const std::size_t start = grid.index(x, y);
      if (!present[start] || visited[start]) continue;
      std::vector<std::size_t> queue{start};
      visited[start] = 1;
      std::vector<Eigen::Vector2d> component;
      for (std::size_t head = 0; head < queue.size(); ++head) {
        const std::size_t id = queue[head];
        const int cy = static_cast<int>(id / static_cast<std::size_t>(grid.width));
        const int cx = static_cast<int>(id % static_cast<std::size_t>(grid.width));
        component.push_back(grid.position(cx, cy));
        for (const auto &neighbor : neighbors) {
          const int nx = cx + neighbor[0];
          const int ny = cy + neighbor[1];
          if (!grid.inBounds(nx, ny)) continue;
          const std::size_t next = grid.index(nx, ny);
          if (present[next] && !visited[next]) {
            visited[next] = 1;
            queue.push_back(next);
          }
        }
      }
      if (component.size() > largest.size()) largest = std::move(component);
    }
  }
  return largest;
}

}  // namespace

ObservationPlanner::ObservationPlanner(const ObservationPlannerConfig &config)
    : config_(config) {}

void ObservationPlanner::reset() { wiped_cells_.clear(); }

void ObservationPlanner::markWiped(const ObservationPlan &plan) {
  wiped_cells_.insert(wiped_cells_.end(), plan.wipeable_cells.begin(),
                      plan.wipeable_cells.end());
}

bool ObservationPlanner::plan(
    const EsdfSnapshot &snapshot, const Eigen::Vector2d &current_xy,
    const std::vector<Eigen::Vector2d> &visited_targets,
    ObservationPlan *plan_out) const {
  if (!plan_out) return false;
  *plan_out = ObservationPlan{};
  if (!snapshot.task_mapping_active) {
    plan_out->reason = "mapping_inactive";
    return false;
  }
  const auto &polygon = snapshot.source_roi_polygon.size() >= 3
                            ? snapshot.source_roi_polygon
                            : snapshot.original_roi_polygon;
  if (polygon.size() < 3) {
    plan_out->reason = "original_polygon_missing";
    return false;
  }
  if (snapshot.mapping_status.motion_state != MotionState::kReadyToMove ||
      !snapshot.mapping_status.ready_to_move) {
    plan_out->reason = "waiting_for_station_commit";
    return false;
  }

  GridView grid;
  ObservationMetrics metrics;
  std::vector<Eigen::Vector2d> wipeable;
  if (!buildGrid(snapshot, config_, snapshot.source_roi_polygon.size() >= 3
                                    ? snapshot.source_roi_polygon
                                    : snapshot.original_roi_polygon,
                 &grid, &metrics, &wipeable)) {
    plan_out->reason = "original_map_unavailable";
    return false;
  }

  std::vector<Eigen::Vector2d> remaining_wipeable;
  for (const auto & cell : wipeable) {
    if (!nearAny(cell, wiped_cells_, 0.5 * config_.candidate_spacing)) {
      remaining_wipeable.push_back(cell);
    }
  }
  remaining_wipeable = largestFreeComponent(grid, remaining_wipeable);
  metrics.wipeable_cells = remaining_wipeable.size();
  if (metrics.original_cells > 0) {
    // Completeness is local to the region that can actually be wiped.  A
    // distant, unrelated unknown corner must not prevent wiping a visible
    // patch.  When no free patch exists yet, use the full original ROI.
    std::size_t near_unknown = 0;
    std::size_t near_total = 0;
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const std::size_t id = grid.index(x, y);
        if (grid.state[id] == 2) {
          const auto point = grid.position(x, y);
          if (remaining_wipeable.empty() ||
              nearAny(point, remaining_wipeable,
                      std::max(config_.arm_reach, 0.30))) {
            ++near_unknown;
            ++near_total;
          }
        } else if (grid.state[id] == 0 && !remaining_wipeable.empty() &&
                   nearAny(grid.position(x, y), remaining_wipeable,
                           std::max(config_.arm_reach, 0.30))) {
          ++near_total;
        }
      }
    }
    metrics.unknown_ratio = near_total == 0
                                ? 1.0
                                : static_cast<double>(near_unknown) /
                                      static_cast<double>(near_total);
  }

  // An opening must be known free in the original wipe band and have a short,
  // known-free entry corridor in the expanded map.  This is a conservative
  // geometric check, not full manipulator reachability/collision planning.
  struct Opening {
    Eigen::Vector2d point;
    Eigen::Vector2d inward;
  };
  std::vector<Opening> openings;
  const Eigen::Vector2d centroid = [&polygon]() {
    Eigen::Vector2d result = Eigen::Vector2d::Zero();
    for (const auto & p : polygon) result += p.cast<double>();
    return result / static_cast<double>(polygon.size());
  }();
  for (int y = 1; y + 1 < grid.height; ++y) {
    for (int x = 1; x + 1 < grid.width; ++x) {
      const Eigen::Vector2d point = grid.position(x, y);
      if (!pointInPolygon(point, polygon) || grid.state[grid.index(x, y)] != 0)
        continue;
      const double boundary_distance = [&]() {
        double distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < polygon.size(); ++i) {
          const Eigen::Vector2d a = polygon[i].cast<double>();
          const Eigen::Vector2d b = polygon[(i + 1) % polygon.size()].cast<double>();
          distance = std::min(distance, distanceToSegment(point, a, b, nullptr));
        }
        return distance;
      }();
      if (boundary_distance > std::max(config_.opening_width, grid.resolution) * 1.5)
        continue;
      const Eigen::Vector2d inward = (centroid - point).normalized();
      if (inward.squaredNorm() <= kEps ||
          !armEntryCorridorFree(snapshot, config_, point, inward))
        continue;
      openings.push_back({point, inward});
    }
  }
  metrics.opening_cells = openings.size();
  metrics.opening_found = !openings.empty();
  metrics.wipeable_region_found =
      remaining_wipeable.size() >= static_cast<std::size_t>(
                                      std::max(1.0, config_.min_wipeable_cells));
  plan_out->metrics = metrics;
  plan_out->wipeable_cells = remaining_wipeable;

  // After at least one wipe, a small remainder is not worth another base
  // relocation.  The caller can finish the task while retaining the evidence
  // that this was a deliberate area-size decision.
  if (!wiped_cells_.empty() && !metrics.wipeable_region_found) {
    plan_out->valid = true;
    plan_out->complete = true;
    plan_out->reason = "remaining_region_below_minimum";
    return true;
  }

  const bool unknown_complete = metrics.unknown_ratio <= config_.max_unknown_ratio;
  const bool enough_region = metrics.wipeable_region_found;
  if (metrics.opening_found && enough_region && unknown_complete) {
    plan_out->valid = true;
    plan_out->complete = true;
    plan_out->reason = "observation_complete";
    return true;
  }

  // Build candidate poses from openings and polygon edge samples.  The score
  // is information gain, with a small travel cost so fewer views win.
  std::vector<ObservationTarget> candidates;
  for (const auto & opening : openings) {
    const Eigen::Vector2d position = opening.point -
                                     opening.inward * config_.candidate_standoff;
    const double distance = (opening.point - position).norm();
    if (distance > config_.sensor_range + config_.arm_reach) continue;
    ObservationTarget candidate;
    candidate.xy = position;
    candidate.yaw = std::atan2(opening.point.y() - position.y(),
                               opening.point.x() - position.x());
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const Eigen::Vector2d cell = grid.position(x, y);
        const double range = (cell - position).norm();
        if (range > config_.sensor_range ||
            std::abs(wrapAngle(std::atan2(cell.y() - position.y(),
                                          cell.x() - position.x()) -
                               candidate.yaw)) > config_.sensor_half_fov)
          continue;
        if (!rayVisible(grid, position, cell, true)) continue;
        const uint8_t state = grid.state[grid.index(x, y)];
        if (state == 2) ++candidate.visible_unknown;
        if (state == 0 && nearAny(cell, remaining_wipeable, grid.resolution * 1.5))
          ++candidate.visible_wipeable;
      }
    }
    candidate.visible_opening = 1;
    candidate.score = static_cast<double>(candidate.visible_unknown) +
                     0.5 * static_cast<double>(candidate.visible_wipeable) +
                     2.0;
    if (nearAny(candidate.xy, visited_targets, config_.candidate_spacing))
      candidate.score *= 0.05;
    candidate.score -= config_.travel_penalty * (candidate.xy - current_xy).norm();
    candidates.push_back(candidate);
  }

  // Before an opening is known, use unknown frontier cells as information
  // targets.  This is the important difference from the old wall-candidate
  // logic: the first view is chosen by expected unknown reduction, then later
  // views are selected only when the map gate has committed the new station.
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (grid.state[grid.index(x, y)] != 2) continue;
      const Eigen::Vector2d point = grid.position(x, y);
      if (!pointInPolygon(point, polygon)) continue;
      const Eigen::Vector2d inward = (centroid - point).normalized();
      const Eigen::Vector2d position = point - inward * config_.candidate_standoff;
      const auto &expanded = snapshot.expanded_geometry;
      if (expanded.size_x <= 0 || expanded.size_y <= 0 ||
          position.x() < expanded.origin.x() ||
          position.x() > expanded.origin.x() + expanded.size_x * expanded.resolution ||
          position.y() < expanded.origin.y() ||
          position.y() > expanded.origin.y() + expanded.size_y * expanded.resolution)
        continue;
      ObservationTarget candidate;
      candidate.xy = position;
      candidate.yaw = std::atan2(point.y() - position.y(),
                                 point.x() - position.x());
      for (int yy = 0; yy < grid.height; ++yy) {
        for (int xx = 0; xx < grid.width; ++xx) {
          const Eigen::Vector2d cell = grid.position(xx, yy);
          if ((cell - position).norm() > config_.sensor_range ||
              std::abs(wrapAngle(std::atan2(cell.y() - position.y(),
                                            cell.x() - position.x()) -
                                 candidate.yaw)) > config_.sensor_half_fov ||
              !rayVisible(grid, position, cell, true))
            continue;
          const uint8_t state = grid.state[grid.index(xx, yy)];
          if (state == 2) ++candidate.visible_unknown;
          if (state == 0 && nearAny(cell, remaining_wipeable,
                                     grid.resolution * 1.5))
            ++candidate.visible_wipeable;
        }
      }
      candidate.score = static_cast<double>(candidate.visible_unknown) +
                        0.5 * static_cast<double>(candidate.visible_wipeable);
      if (nearAny(candidate.xy, visited_targets, config_.candidate_spacing))
        candidate.score *= 0.05;
      candidate.score -= config_.travel_penalty * (candidate.xy - current_xy).norm();
      if (candidate.visible_unknown > 0) candidates.push_back(candidate);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const ObservationTarget &a, const ObservationTarget &b) {
              return a.score > b.score;
            });
  if (candidates.size() > config_.max_candidates) {
    candidates.resize(config_.max_candidates);
  }
  if (candidates.empty()) {
    metrics.theoretically_unobservable = true;
    plan_out->metrics = metrics;
    plan_out->reason = "no_visible_candidate";
    // If there is no line of sight at all, no amount of repeated observation
    // can improve the map.  Allow wiping the known region while reporting the
    // residual unknown area to the caller.
    if (metrics.opening_found && enough_region) {
      plan_out->valid = true;
      plan_out->complete = true;
      plan_out->reason = "complete_except_theoretically_unobservable";
      return true;
    }
    return false;
  }
  const auto &selected = candidates.front();
  if (selected.visible_unknown == 0 && metrics.unknown_ratio > config_.max_unknown_ratio) {
    metrics.theoretically_unobservable = true;
  }
  plan_out->valid = selected.visible_unknown > 0 ||
                    selected.visible_wipeable > 0 || !metrics.opening_found;
  plan_out->target = selected;
  plan_out->metrics = metrics;
  plan_out->reason = selected.visible_unknown == 0
                         ? "best_candidate_no_new_unknown"
                         : "observe_unknown_frontier";
  return plan_out->valid;
}

}  // namespace plan_env

