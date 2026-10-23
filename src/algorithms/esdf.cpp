#include "plan_env/esdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <omp.h>

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
    std::fill_n(line.output.begin(), n, kInfinity);
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

void transformVolume(std::vector<float> *distance, int sx, int sy, int sz,
                     int thread_limit) {
  const auto address = [sy, sz](int x, int y, int z) {
    return (static_cast<std::size_t>(x) * sy + y) * sz + z;
  };
  const int capacity = std::max({sx, sy, sz});
  // Z is contiguous, as in the legacy transform. Start there before strided
  // Y/X. Small maps run serially to avoid team startup costs. Axis barriers are
  // required because each pass consumes the complete preceding pass.
  const int threads =
      std::max(1, std::min({thread_limit, omp_get_max_threads(),
                            static_cast<int>(std::max<std::size_t>(
                                1, distance->size() / 32768))}));
#pragma omp parallel num_threads(threads) if (threads > 1)
  {
    LineWorkspace line(capacity);
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
  }
}

}  // namespace

bool EsdfVolume::configure(const GridGeometry &geometry,
                           const EsdfConfig &config) {
  if (!validGeometry(geometry) || !std::isfinite(config.max_distance) ||
      config.max_distance < 0.0f || std::isnan(config.unknown_z_min) ||
      std::isnan(config.unknown_z_max) ||
      config.unknown_z_min > config.unknown_z_max || config.omp_threads < 1) {
    return false;
  }
  geometry_ = geometry;
  config_ = config;
  const std::size_t count = static_cast<std::size_t>(geometry.size_x) *
                            static_cast<std::size_t>(geometry.size_y) *
                            static_cast<std::size_t>(geometry.size_z);
  voxels_.assign(count, EsdfVoxel{});
  distance_scratch_.clear();
  std::vector<float>().swap(positive_z_);
  std::vector<float>().swap(positive_y_);
  std::vector<float>().swap(negative_z_);
  std::vector<float>().swap(negative_y_);
  source_mask_.clear();
  sources_valid_ = false;
  local_cache_valid_ = false;
  last_updated_lines_ = 0;
  return true;
}

void EsdfVolume::clear() {
  std::vector<float>().swap(positive_z_);
  std::vector<float>().swap(positive_y_);
  std::vector<float>().swap(negative_z_);
  std::vector<float>().swap(negative_y_);
  geometry_ = GridGeometry{};
  voxels_.clear();
  std::vector<float>().swap(distance_scratch_);
  std::vector<unsigned char>().swap(source_mask_);
  sources_valid_ = false;
  local_cache_valid_ = false;
  last_updated_lines_ = 0;
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
  sources_valid_ = false;
  local_cache_valid_ = false;
  last_updated_lines_ = 0;
  return voxels_;
}

bool EsdfVolume::compute(const std::vector<OccupancyVoxel> &occupancy,
                         const occupancy::LogOddsModel &model) {
  return computeImpl(occupancy, model, false, nullptr);
}

bool EsdfVolume::computeIfChanged(const std::vector<OccupancyVoxel> &occupancy,
                                  const occupancy::LogOddsModel &model,
                                  bool *recomputed) {
  return computeImpl(occupancy, model, true, recomputed);
}

bool EsdfVolume::computeImpl(const std::vector<OccupancyVoxel> &occupancy,
                             const occupancy::LogOddsModel &model, bool reuse,
                             bool *recomputed) {
  last_updated_lines_ = 0;
  if (recomputed) {
    *recomputed = false;
  }
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
  bool changed = !sources_valid_ || source_mask_.size() != count;
  source_mask_.resize(count);
  std::size_t source_count = 0;
  std::vector<int> changed_columns;
  int previous_column = -1;
  for (std::size_t i = 0; i < count; ++i) {
    const auto state = model.state(occupancy[i]);
    const int z = static_cast<int>(i % static_cast<std::size_t>(sz));
    const unsigned char source =
        state == OccupancyState::OCCUPIED ||
        (config_.unknown_is_occupied && state == OccupancyState::UNKNOWN &&
         z >= z_min && z <= z_max);
    if (source_mask_[i] != source) {
      changed = true;
      const int column = static_cast<int>(i / sz);
      if (column != previous_column) {
        changed_columns.push_back(column);
        previous_column = column;
      }
    }
    source_mask_[i] = source;
    source_count += source;
  }
  if (reuse && !changed) {
    return true;
  }
  if (reuse && config_.local_update) {
    updateLocal(changed_columns, !local_cache_valid_);
    sources_valid_ = true;
    if (recomputed) {
      *recomputed = true;
    }
    return true;
  }
  local_cache_valid_ = false;
  // Uniform fields need no distance transform, including the initial unknown
  // volume. Preserve signed/unsigned and no-source max_distance semantics.
  if (source_count == 0 || source_count == count) {
    const float value =
        source_count == 0
            ? config_.max_distance
            : (config_.signed_distance ? -config_.max_distance : 0.0f);
    for (auto &voxel : voxels_) {
      voxel.distance = value;
    }
  } else {
    last_updated_lines_ =
        static_cast<std::size_t>(sx * sy + sx * sz + sy * sz) *
        (config_.signed_distance ? 2 : 1);
    distance_scratch_.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      distance_scratch_[i] = source_mask_[i] ? 0.0f : kInfinity;
    }
    transformVolume(&distance_scratch_, sx, sy, sz, config_.omp_threads);
    for (std::size_t i = 0; i < count; ++i) {
      voxels_[i].distance =
          std::min(config_.max_distance,
                   std::sqrt(distance_scratch_[i]) * geometry_.resolution);
    }
    if (config_.signed_distance) {
      // Reuse the positive-pass buffer; its output is already in voxels_.
      for (std::size_t i = 0; i < count; ++i) {
        distance_scratch_[i] = source_mask_[i] ? kInfinity : 0.0f;
      }
      transformVolume(&distance_scratch_, sx, sy, sz, config_.omp_threads);
      for (std::size_t i = 0; i < count; ++i) {
        if (source_mask_[i]) {
          voxels_[i].distance =
              -std::min(config_.max_distance,
                        std::sqrt(distance_scratch_[i]) * geometry_.resolution);
        }
      }
    }
  }
  sources_valid_ = true;
  if (recomputed) {
    *recomputed = true;
  }
  return true;
}

// Only changed source columns enter the first pass. Differences in each
// intermediate squared field activate dependent lines in the next pass. Every
// activated line includes its entire axis, so sources outside the changed AABB
// and removals of the nearest source are handled exactly, without stale edges.
void EsdfVolume::updateLocal(const std::vector<int> &changed_columns,
                             bool initialize) {
  const int sx = geometry_.size_x, sy = geometry_.size_y, sz = geometry_.size_z;
  const std::size_t count = voxels_.size();
  const int capacity = std::max({sx, sy, sz});
  std::vector<int> columns = changed_columns;
  if (initialize) {
    // Full and incremental transforms do not need their scratch simultaneously.
    std::vector<float>().swap(distance_scratch_);
    positive_z_.assign(count, kInfinity);
    positive_y_.assign(count, kInfinity);
    if (config_.signed_distance) {
      negative_z_.assign(count, kInfinity);
      negative_y_.assign(count, kInfinity);
    }
    columns.resize(sx * sy);
    for (int i = 0; i < sx * sy; ++i) {
      columns[i] = i;
    }
  }
  // Reuse one team per sign; only dirty line lists are visited. Atomic flag
  // writes join dependencies from distinct lines, with barriers between axes.
  for (int sign = 0; sign < (config_.signed_distance ? 2 : 1); ++sign) {
    auto &stage_z = sign == 0 ? positive_z_ : negative_z_;
    auto &stage_y = sign == 0 ? positive_y_ : negative_y_;
    std::vector<unsigned char> dirty_y(sx * sz, initialize ? 1 : 0);
    std::vector<unsigned char> dirty_x(sy * sz, initialize ? 1 : 0);
    std::vector<int> lines_y, lines_x;
    const int threads = std::max(
        1, std::min({config_.omp_threads, omp_get_max_threads(),
                     static_cast<int>(std::max<std::size_t>(
                         1, columns.size() * static_cast<std::size_t>(sz) /
                                32768))}));
#pragma omp parallel num_threads(threads) if (threads > 1)
    {
      LineWorkspace line(capacity);
#pragma omp for schedule(static)
      for (std::size_t c = 0; c < columns.size(); ++c) {
        const int column = columns[c];
        const std::size_t base = static_cast<std::size_t>(column) * sz;
        for (int z = 0; z < sz; ++z) {
          const bool source = source_mask_[base + z] != 0;
          line.input[z] = (source != (sign != 0)) ? 0.0f : kInfinity;
        }
        transformLine(line, sz);
        for (int z = 0; z < sz; ++z) {
          if (stage_z[base + z] != line.output[z]) {
            stage_z[base + z] = line.output[z];
            const int next = (column / sy) * sz + z;
#pragma omp atomic write
            dirty_y[next] = 1;
          }
        }
      }
#pragma omp single
      {
        for (int i = 0; i < sx * sz; ++i) {
          if (dirty_y[i]) {
            lines_y.push_back(i);
          }
        }
      }
#pragma omp for schedule(static)
      for (std::size_t c = 0; c < lines_y.size(); ++c) {
        const int x = lines_y[c] / sz, z = lines_y[c] % sz;
        for (int y = 0; y < sy; ++y) {
          line.input[y] =
              stage_z[(static_cast<std::size_t>(x) * sy + y) * sz + z];
        }
        transformLine(line, sy);
        for (int y = 0; y < sy; ++y) {
          const std::size_t i = (static_cast<std::size_t>(x) * sy + y) * sz + z;
          if (stage_y[i] != line.output[y]) {
            stage_y[i] = line.output[y];
            const int next = y * sz + z;
#pragma omp atomic write
            dirty_x[next] = 1;
          }
        }
      }
#pragma omp single
      {
        for (int i = 0; i < sy * sz; ++i) {
          if (dirty_x[i]) {
            lines_x.push_back(i);
          }
        }
      }
#pragma omp for schedule(static)
      for (std::size_t c = 0; c < lines_x.size(); ++c) {
        const int y = lines_x[c] / sz, z = lines_x[c] % sz;
        for (int x = 0; x < sx; ++x) {
          line.input[x] =
              stage_y[(static_cast<std::size_t>(x) * sy + y) * sz + z];
        }
        transformLine(line, sx);
        for (int x = 0; x < sx; ++x) {
          const std::size_t i = (static_cast<std::size_t>(x) * sy + y) * sz + z;
          // A sign transition always changes this sign's squared field from
          // zero to positive (or vice versa), hence activates this output line.
          if (!config_.signed_distance ||
              (source_mask_[i] != 0) == (sign != 0)) {
            const float distance =
                std::min(config_.max_distance,
                         std::sqrt(line.output[x]) * geometry_.resolution);
            voxels_[i].distance = sign == 0 ? distance : -distance;
          }
        }
      }
    }
    last_updated_lines_ += columns.size() + lines_y.size() + lines_x.size();
  }
  local_cache_valid_ = true;
}

bool EsdfVolume::compute(const std::vector<OccupancyVoxel> &occupancy) {
  const occupancy::LogOddsModel default_model;
  return compute(occupancy, default_model);
}

}  // namespace esdf

}  // namespace plan_env
