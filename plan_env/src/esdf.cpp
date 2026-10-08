#include "plan_env/esdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_env {

namespace esdf {
namespace {

constexpr float kInfinity = std::numeric_limits<float>::infinity();

bool validGeometry(const GridGeometry &geometry) {
  return std::isfinite(geometry.resolution) && geometry.resolution > 0.0f &&
         geometry.size_x > 0 && geometry.size_y > 0 && geometry.size_z > 0;
}

// Felzenszwalb and Huttenlocher's exact 1D squared distance transform.
void transformLine(const std::vector<float> &input,
                   std::vector<float> *output) {
  const int n = static_cast<int>(input.size());
  output->assign(input.size(), kInfinity);
  if (n == 0) {
    return;
  }
  std::vector<int> envelope(static_cast<std::size_t>(n));
  std::vector<float> boundaries(static_cast<std::size_t>(n) + 1, 0.0f);
  int envelope_size = -1;
  for (int q = 0; q < n; ++q) {
    if (!std::isfinite(input[q])) {
      continue;
    }
    if (envelope_size < 0) {
      envelope_size = 0;
      envelope[0] = q;
      boundaries[0] = -kInfinity;
      boundaries[1] = kInfinity;
      continue;
    }
    float intersection = 0.0f;
    do {
      const int v = envelope[envelope_size];
      intersection = ((input[q] + static_cast<float>(q) * q) -
                      (input[v] + static_cast<float>(v) * v)) /
                     (2.0f * static_cast<float>(q - v));
      if (intersection <= boundaries[envelope_size]) {
        --envelope_size;
      }
    } while (envelope_size >= 0 && intersection <= boundaries[envelope_size]);
    ++envelope_size;
    envelope[envelope_size] = q;
    boundaries[envelope_size] = intersection;
    boundaries[envelope_size + 1] = kInfinity;
  }
  if (envelope_size < 0) {
    return;
  }
  int k = 0;
  for (int q = 0; q < n; ++q) {
    while (boundaries[k + 1] < static_cast<float>(q)) {
      ++k;
    }
    const float delta = static_cast<float>(q - envelope[k]);
    (*output)[static_cast<std::size_t>(q)] = delta * delta + input[envelope[k]];
  }
}

void transformVolume(std::vector<float> *distance, int sx, int sy, int sz) {
  const auto address = [sy, sz](int x, int y, int z) {
    return static_cast<std::size_t>(x * sy * sz + y * sz + z);
  };
  std::vector<float> input;
  std::vector<float> output;
  input.reserve(static_cast<std::size_t>(std::max(sx, std::max(sy, sz))));
  output.reserve(input.capacity());
  for (int y = 0; y < sy; ++y) {
    for (int z = 0; z < sz; ++z) {
      input.clear();
      for (int x = 0; x < sx; ++x) {
        input.push_back((*distance)[address(x, y, z)]);
      }
      transformLine(input, &output);
      for (int x = 0; x < sx; ++x) {
        (*distance)[address(x, y, z)] = output[x];
      }
    }
  }
  for (int x = 0; x < sx; ++x) {
    for (int z = 0; z < sz; ++z) {
      input.clear();
      for (int y = 0; y < sy; ++y) {
        input.push_back((*distance)[address(x, y, z)]);
      }
      transformLine(input, &output);
      for (int y = 0; y < sy; ++y) {
        (*distance)[address(x, y, z)] = output[y];
      }
    }
  }
  for (int x = 0; x < sx; ++x) {
    for (int y = 0; y < sy; ++y) {
      input.clear();
      for (int z = 0; z < sz; ++z) {
        input.push_back((*distance)[address(x, y, z)]);
      }
      transformLine(input, &output);
      for (int z = 0; z < sz; ++z) {
        (*distance)[address(x, y, z)] = output[z];
      }
    }
  }
}

}  // namespace

bool EsdfVolume::configure(const GridGeometry &geometry,
                           const EsdfConfig &config) {
  if (!validGeometry(geometry) || !std::isfinite(config.max_distance) ||
      config.max_distance < 0.0f) {
    return false;
  }
  geometry_ = geometry;
  config_ = config;
  const std::size_t count = static_cast<std::size_t>(geometry.size_x) *
                            static_cast<std::size_t>(geometry.size_y) *
                            static_cast<std::size_t>(geometry.size_z);
  voxels_.assign(count, EsdfVoxel{});
  return true;
}

void EsdfVolume::clear() {
  geometry_ = GridGeometry{};
  voxels_.clear();
}

bool EsdfVolume::valid() const {
  return validGeometry(geometry_) && !voxels_.empty();
}

const GridGeometry &EsdfVolume::geometry() const {
  return geometry_;
}

const std::vector<EsdfVoxel> &EsdfVolume::voxels() const {
  return voxels_;
}

std::vector<EsdfVoxel> &EsdfVolume::voxels() {
  return voxels_;
}

bool EsdfVolume::compute(const std::vector<OccupancyVoxel> &occupancy,
                         const occupancy::LogOddsModel &model) {
  if (!valid() || occupancy.size() != voxels_.size()) {
    return false;
  }

  const int sx = geometry_.size_x;
  const int sy = geometry_.size_y;
  const int sz = geometry_.size_z;
  const std::size_t count = voxels_.size();
  std::vector<float> distance(count, kInfinity);
  for (std::size_t i = 0; i < count; ++i) {
    const OccupancyState state = model.state(occupancy[i]);
    if (state == OccupancyState::OCCUPIED ||
        (config_.unknown_is_occupied && state == OccupancyState::UNKNOWN)) {
      distance[i] = 0.0f;
    }
  }

  transformVolume(&distance, sx, sy, sz);

  for (std::size_t i = 0; i < count; ++i) {
    float value = std::isfinite(distance[i])
                      ? std::sqrt(distance[i]) * geometry_.resolution
                      : config_.max_distance;
    voxels_[i].distance = std::min(config_.max_distance, value);
  }

  if (config_.signed_distance) {
    std::vector<float> distance_to_free(count, kInfinity);
    for (std::size_t i = 0; i < count; ++i) {
      const OccupancyState state = model.state(occupancy[i]);
      if (state == OccupancyState::FREE ||
          (!config_.unknown_is_occupied && state == OccupancyState::UNKNOWN)) {
        distance_to_free[i] = 0.0f;
      }
    }
    transformVolume(&distance_to_free, sx, sy, sz);
    for (std::size_t i = 0; i < count; ++i) {
      const OccupancyState state = model.state(occupancy[i]);
      if (state == OccupancyState::OCCUPIED ||
          (config_.unknown_is_occupied && state == OccupancyState::UNKNOWN)) {
        const float value =
            std::isfinite(distance_to_free[i])
                ? std::sqrt(distance_to_free[i]) * geometry_.resolution
                : config_.max_distance;
        voxels_[i].distance = -std::min(config_.max_distance, value);
      }
    }
  }
  return true;
}

bool EsdfVolume::compute(const std::vector<OccupancyVoxel> &occupancy) {
  const occupancy::LogOddsModel default_model;
  return compute(occupancy, default_model);
}

}  // namespace esdf

}  // namespace plan_env
