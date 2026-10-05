#include "plan_env/occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_env {

namespace occupancy {
namespace {

bool validConfig(const LogOddsConfig &config) {
  return std::isfinite(config.hit_log_odds) &&
         std::isfinite(config.miss_log_odds) &&
         std::isfinite(config.min_log_odds) &&
         std::isfinite(config.max_log_odds) &&
         std::isfinite(config.occupied_threshold) &&
         std::isfinite(config.free_threshold) &&
         config.min_log_odds <= config.max_log_odds &&
         config.free_threshold <= config.occupied_threshold &&
         config.free_threshold >= config.min_log_odds &&
         config.free_threshold <= config.max_log_odds &&
         config.occupied_threshold >= config.min_log_odds &&
         config.occupied_threshold <= config.max_log_odds;
}

} // namespace

LogOddsModel::LogOddsModel(const LogOddsConfig &config) {
  if (!configure(config)) {
    config_ = LogOddsConfig{};
  }
}

bool LogOddsModel::configure(const LogOddsConfig &config) {
  if (!validConfig(config)) {
    return false;
  }
  config_ = config;
  return true;
}

const LogOddsConfig &LogOddsModel::config() const { return config_; }

void LogOddsModel::update(OccupancyVoxel &voxel, const bool hit) const {
  if (hit) {
    voxel.hit_count = static_cast<uint16_t>(std::min<uint32_t>(
        std::numeric_limits<uint16_t>::max(),
        static_cast<uint32_t>(voxel.hit_count) + 1));
  } else {
    voxel.miss_count = static_cast<uint16_t>(std::min<uint32_t>(
        std::numeric_limits<uint16_t>::max(),
        static_cast<uint32_t>(voxel.miss_count) + 1));
  }

  const float increment = hit ? config_.hit_log_odds : config_.miss_log_odds;
  if (!std::isfinite(voxel.log_odds)) {
    voxel.log_odds = 0.0f;
  }
  voxel.log_odds = std::max(
      config_.min_log_odds,
      std::min(config_.max_log_odds, voxel.log_odds + increment));
}

float LogOddsModel::probability(const OccupancyVoxel &voxel) const {
  if (!std::isfinite(voxel.log_odds)) {
    return 0.5f;
  }
  // The clamp avoids overflow in exp for corrupted external snapshot data.
  const float log_odds = std::max(-80.0f, std::min(80.0f, voxel.log_odds));
  return 1.0f / (1.0f + std::exp(-log_odds));
}

OccupancyState LogOddsModel::state(const OccupancyVoxel &voxel) const {
  if (!std::isfinite(voxel.log_odds) ||
      (voxel.log_odds > config_.free_threshold &&
       voxel.log_odds < config_.occupied_threshold)) {
    return OccupancyState::UNKNOWN;
  }
  if (voxel.log_odds > config_.occupied_threshold) {
    return OccupancyState::OCCUPIED;
  }
  if (voxel.log_odds < config_.free_threshold) {
    return OccupancyState::FREE;
  }
  // At the shared p=0.5 boundary the evidence is ambiguous.
  return OccupancyState::UNKNOWN;
}

bool LogOddsModel::isOccupied(const OccupancyVoxel &voxel) const {
  return state(voxel) == OccupancyState::OCCUPIED;
}

} // namespace occupancy

}  // namespace plan_env
