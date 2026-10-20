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

// One scratch set per OpenMP worker, reused across all lines and axes.
struct LineWorkspace {
  explicit LineWorkspace(int capacity)
      : input(capacity), output(capacity), envelope(capacity),
        boundaries(capacity + 1) {}
  std::vector<float> input;
  std::vector<float> output;
  std::vector<int> envelope;
  std::vector<float> boundaries;
};

// Felzenszwalb and Huttenlocher's exact 1D squared distance transform.
void transformLine(LineWorkspace &line, const int n) {
  std::fill_n(line.output.begin(), n, kInfinity);
  int last = -1;
  for (int q = 0; q < n; ++q) {
    if (!std::isfinite(line.input[q])) {
      continue;
    }
    float intersection = -kInfinity;
    while (last >= 0) {
      const int v = line.envelope[last];
      intersection = ((line.input[q] + static_cast<float>(q) * q) -
                      (line.input[v] + static_cast<float>(v) * v)) /
                     (2.0f * static_cast<float>(q - v));
      if (intersection > line.boundaries[last]) {
        break;
      }
      --last;
    }
    ++last;
    line.envelope[last] = q;
    line.boundaries[last] = last == 0 ? -kInfinity : intersection;
    line.boundaries[last + 1] = kInfinity;
  }
  if (last < 0) {
    return;
  }
  int k = 0;
  for (int q = 0; q < n; ++q) {
    while (line.boundaries[k + 1] < static_cast<float>(q)) {
      ++k;
    }
    const float delta = static_cast<float>(q - line.envelope[k]);
    line.output[q] = delta * delta + line.input[line.envelope[k]];
  }
}

void transformVolume(std::vector<float> *distance, int sx, int sy, int sz) {
  const auto address = [sy, sz](int x, int y, int z) {
    return (static_cast<std::size_t>(x) * sy + y) * sz + z;
  };
  const int capacity = std::max({sx, sy, sz});
  // Small maps run serially to avoid team startup costs. Axis barriers are
  // required because each pass consumes the complete preceding pass.
#pragma omp parallel if (distance->size() >= 32768)
  {
    LineWorkspace line(capacity);
#pragma omp for collapse(2) schedule(static)
    for (int y = 0; y < sy; ++y) {
      for (int z = 0; z < sz; ++z) {
        for (int x = 0; x < sx; ++x) {
          line.input[x] = (*distance)[address(x, y, z)];
        }
        transformLine(line, sx);
        for (int x = 0; x < sx; ++x) {
          (*distance)[address(x, y, z)] = line.output[x];
        }
      }
    }
#pragma omp for collapse(2) schedule(static)
    for (int x = 0; x < sx; ++x) {
      for (int z = 0; z < sz; ++z) {
        for (int y = 0; y < sy; ++y) {
          line.input[y] = (*distance)[address(x, y, z)];
        }
        transformLine(line, sy);
        for (int y = 0; y < sy; ++y) {
          (*distance)[address(x, y, z)] = line.output[y];
        }
      }
    }
#pragma omp for collapse(2) schedule(static)
    for (int x = 0; x < sx; ++x) {
      for (int y = 0; y < sy; ++y) {
        for (int z = 0; z < sz; ++z) {
          line.input[z] = (*distance)[address(x, y, z)];
        }
        transformLine(line, sz);
        for (int z = 0; z < sz; ++z) {
          (*distance)[address(x, y, z)] = line.output[z];
        }
      }
    }
  }
}

}  // namespace

bool EsdfVolume::configure(const GridGeometry &geometry,
                           const EsdfConfig &config) {
  if (!validGeometry(geometry) || !std::isfinite(config.max_distance) ||
      config.max_distance < 0.0f || std::isnan(config.unknown_z_min) ||
      std::isnan(config.unknown_z_max) ||
      config.unknown_z_min > config.unknown_z_max) {
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
  const double z_min = std::floor(
      (static_cast<double>(config_.unknown_z_min) - geometry_.origin.z()) /
      geometry_.resolution);
  const double z_max = std::floor(
      (static_cast<double>(config_.unknown_z_max) - geometry_.origin.z()) /
      geometry_.resolution);
  const auto is_source = [&](std::size_t i) {
    const auto state = model.state(occupancy[i]);
    const int z = static_cast<int>(i % static_cast<std::size_t>(sz));
    return state == OccupancyState::OCCUPIED ||
           (config_.unknown_is_occupied && state == OccupancyState::UNKNOWN &&
            z >= z_min && z <= z_max);
  };
  std::vector<float> distance(count, kInfinity);
  for (std::size_t i = 0; i < count; ++i) {
    if (is_source(i)) {
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
      if (!is_source(i)) {
        distance_to_free[i] = 0.0f;
      }
    }
    transformVolume(&distance_to_free, sx, sy, sz);
    for (std::size_t i = 0; i < count; ++i) {
      if (is_source(i)) {
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
