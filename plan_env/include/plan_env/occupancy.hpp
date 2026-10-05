#pragma once

#include <cstdint>

namespace plan_env {

// A log-odds voxel is intentionally a small value type so snapshots can be
// copied without depending on the fusion implementation.
struct OccupancyVoxel {
  float log_odds = 0.0f;
  uint16_t hit_count = 0;
  uint16_t miss_count = 0;
};

enum class OccupancyState { FREE, OCCUPIED, UNKNOWN };

namespace occupancy {

struct LogOddsConfig {
  float hit_log_odds = 0.85f;
  float miss_log_odds = -0.40f;
  float min_log_odds = -5.0f;
  float max_log_odds = 5.0f;
  // Zero is the conventional p=0.5 decision boundary. A zero voxel remains
  // UNKNOWN until the first observation is fused.
  float occupied_threshold = 0.0f;
  float free_threshold = 0.0f;
};

class LogOddsModel {
public:
  explicit LogOddsModel(const LogOddsConfig &config = LogOddsConfig{});

  bool configure(const LogOddsConfig &config);
  const LogOddsConfig &config() const;

  void update(OccupancyVoxel &voxel, bool hit) const;
  float probability(const OccupancyVoxel &voxel) const;
  OccupancyState state(const OccupancyVoxel &voxel) const;
  bool isOccupied(const OccupancyVoxel &voxel) const;

private:
  LogOddsConfig config_;
};

} // namespace occupancy

}  // namespace plan_env
