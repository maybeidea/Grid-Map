#pragma once

#include <cstdint>

namespace plan_env {

struct HeightStatus {
  static constexpr std::uint8_t kOpen = 0;
  static constexpr std::uint8_t kLowClearance = 1;
  static constexpr std::uint8_t kTooLow = 2;
  static constexpr std::uint8_t kBlockedAtFloor = 3;
  static constexpr std::uint8_t kNotFloor = 4;
  static constexpr std::uint8_t kUnknown = 5;
};

struct HeightBandParams {
  double ground_height{0.0};
  double floor_tol{0.02};
  double blocked_at_floor_threshold{0.04};
  double z_tcp_min{0.07};
  double ceil_threshold_open{0.50};
};

inline bool occupiedCountsForHeight(const double z_rel,
                                    const HeightBandParams &params) {
  return z_rel > params.floor_tol;
}

inline std::uint8_t statusFromLowestOccupied(const double z_rel,
                                             const HeightBandParams &params) {
  if (z_rel <= params.floor_tol) return HeightStatus::kOpen;
  if (z_rel <= params.blocked_at_floor_threshold) {
    return HeightStatus::kBlockedAtFloor;
  }
  if (z_rel <= params.z_tcp_min) return HeightStatus::kTooLow;
  if (z_rel < params.ceil_threshold_open) return HeightStatus::kLowClearance;
  return HeightStatus::kOpen;
}

inline std::uint8_t classifyHeightColumn(const bool has_occupied,
                                         const bool has_free,
                                         const double min_z_rel,
                                         const HeightBandParams &params) {
  if (has_occupied) return statusFromLowestOccupied(min_z_rel, params);
  if (has_free) return HeightStatus::kOpen;
  return HeightStatus::kUnknown;
}

inline float columnKnownFraction(const int known, const int total) {
  return total > 0 ? static_cast<float>(known) / static_cast<float>(total)
                   : 0.0f;
}

}  // namespace plan_env
