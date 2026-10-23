#include "plan_env/esdf.hpp"
#include "plan_env/ray_integration.hpp"
#include "plan_env/ray_casting.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <random>

namespace plan_env {
namespace {

TEST(EsdfTest, SignedDistanceMatchesBruteForceOnRandomAndUniformVolumes) {
  GridGeometry geometry;
  geometry.resolution = 0.125f;
  geometry.size_x = 7;
  geometry.size_y = 5;
  geometry.size_z = 4;
  const int count = geometry.size_x * geometry.size_y * geometry.size_z;
  auto index = [&](int i) {
    return Eigen::Vector3i(i / (geometry.size_y * geometry.size_z),
                           (i / geometry.size_z) % geometry.size_y,
                           i % geometry.size_z);
  };
  occupancy::LogOddsModel model;
  std::mt19937 random(42);
  for (int trial = 0; trial < 22; ++trial) {
    std::vector<OccupancyVoxel> voxels(count);
    std::vector<bool> occupied(count);
    for (int i = 0; i < count; ++i) {
      occupied[i] = trial == 0 || (trial > 1 && random() % 4 == 0);
      model.update(voxels[i], occupied[i]);
    }
    esdf::EsdfConfig config;
    config.max_distance = 2.0f;
    config.unknown_is_occupied = false;
    esdf::EsdfVolume volume;
    ASSERT_TRUE(volume.configure(geometry, config));
    ASSERT_TRUE(volume.compute(voxels, model));
    for (int i = 0; i < count; ++i) {
      float expected = config.max_distance;
      for (int j = 0; j < count; ++j) {
        if (occupied[i] != occupied[j]) {
          expected =
              std::min(expected, (index(i) - index(j)).cast<float>().norm() *
                                     geometry.resolution);
        }
      }
      if (occupied[i]) {
        expected = -expected;
      }
      EXPECT_NEAR(expected, volume.voxels()[i].distance, 1e-6)
          << "trial=" << trial << " voxel=" << i;
    }
  }
}

TEST(EsdfTest, ParallelAxesMatchNearestObstacleReference) {
  GridGeometry geometry;
  geometry.resolution = 0.05f;
  geometry.size_x = 40;
  geometry.size_y = 32;
  geometry.size_z = 30;
  std::vector<Eigen::Vector3i> obstacles{{1, 2, 3}, {21, 25, 12}, {39, 0, 29}};
  std::vector<OccupancyVoxel> voxels(40 * 32 * 30);
  occupancy::LogOddsModel model;
  for (auto &voxel : voxels) {
    model.update(voxel, false);
  }
  for (const auto &id : obstacles) {
    auto &voxel = voxels[(id.x() * 32 + id.y()) * 30 + id.z()];
    model.update(voxel, true);
  }
  esdf::EsdfConfig config;
  config.signed_distance = false;
  config.unknown_is_occupied = false;
  esdf::EsdfVolume volume;
  ASSERT_TRUE(volume.configure(geometry, config));
  ASSERT_TRUE(volume.compute(voxels, model));
  for (int x = 0; x < 40; ++x) {
    for (int y = 0; y < 32; ++y) {
      for (int z = 0; z < 30; ++z) {
        float expected = config.max_distance;
        for (const auto &obstacle : obstacles) {
          expected = std::min(
              expected,
              (Eigen::Vector3i(x, y, z) - obstacle).cast<float>().norm() *
                  geometry.resolution);
        }
        EXPECT_NEAR(expected, volume.voxels()[(x * 32 + y) * 30 + z].distance,
                    1e-6);
      }
    }
  }
}

TEST(EsdfTest, CachedSourcesRemainExactAcrossEvidenceAndPolicyChanges) {
  GridGeometry geometry;
  geometry.resolution = 0.1f;
  geometry.size_x = 7; geometry.size_y = 5; geometry.size_z = 4;
  occupancy::LogOddsModel model;
  for (bool signed_distance : {false, true}) {
    for (bool unknown_occupied : {false, true}) {
      esdf::EsdfConfig config;
      config.signed_distance = signed_distance;
      config.unknown_is_occupied = unknown_occupied;
      config.unknown_z_min = 0.1f;
      config.unknown_z_max = 0.2f;
      esdf::EsdfVolume cached, reference;
      ASSERT_TRUE(cached.configure(geometry, config));
      ASSERT_TRUE(reference.configure(geometry, config));
      std::vector<OccupancyVoxel> occupancy(140);
      std::mt19937 random(19);
      for (int frame = 0; frame < 40; ++frame) {
        for (int j = 0; j < 10; ++j) {
          occupancy[random() % occupancy.size()].log_odds =
              static_cast<int>(random() % 3) - 1;
        }
        bool recomputed = false;
        ASSERT_TRUE(cached.computeIfChanged(occupancy, model, &recomputed));
        ASSERT_TRUE(reference.compute(occupancy, model));
        const auto &actual = static_cast<const esdf::EsdfVolume &>(cached).voxels();
        for (std::size_t i = 0; i < actual.size(); ++i) {
          EXPECT_NEAR(reference.voxels()[i].distance, actual[i].distance, 1e-6);
        }
        ASSERT_TRUE(cached.computeIfChanged(occupancy, model, &recomputed));
        EXPECT_FALSE(recomputed);
        // Confidence changes without crossing a decision boundary do not
        // change the ESDF, but are still retained in occupancy/snapshots.
        for (auto &voxel : occupancy) voxel.log_odds *= 2;
        ASSERT_TRUE(cached.computeIfChanged(occupancy, model, &recomputed));
        EXPECT_FALSE(recomputed);
      }
      // Mutable access and reconfiguration must invalidate cached sources.
      cached.voxels()[0].distance = -123;
      bool recomputed = false;
      ASSERT_TRUE(cached.computeIfChanged(occupancy, model, &recomputed));
      EXPECT_TRUE(recomputed);
      ASSERT_TRUE(cached.configure(geometry, config));
      ASSERT_TRUE(cached.computeIfChanged(occupancy, model, &recomputed));
      EXPECT_TRUE(recomputed);
      occupancy::LogOddsConfig thresholds;
      thresholds.occupied_threshold = 2.f;
      thresholds.free_threshold = -2.f;
      occupancy::LogOddsModel changed_model(thresholds);
      ASSERT_TRUE(cached.computeIfChanged(occupancy, changed_model));
      ASSERT_TRUE(reference.compute(occupancy, changed_model));
      for (std::size_t i = 0; i < occupancy.size(); ++i) {
        EXPECT_NEAR(reference.voxels()[i].distance, cached.voxels()[i].distance, 1e-6);
      }
    }
  }
}

TEST(EsdfTest, ThreadLimitsAndUniformUnsignedFields) {
  GridGeometry geometry;
  geometry.resolution = 0.1f;
  geometry.size_x = 40; geometry.size_y = 64; geometry.size_z = 30;
  std::vector<OccupancyVoxel> occupancy(40 * 64 * 30);
  for (auto &v : occupancy) v.log_odds = -1;
  occupancy[123].log_odds = 1;
  occupancy::LogOddsModel model;
  esdf::EsdfVolume serial, parallel;
  esdf::EsdfConfig config;
  config.omp_threads = 1;
  ASSERT_TRUE(serial.configure(geometry, config));
  ASSERT_TRUE(serial.compute(occupancy, model));
  config.omp_threads = 2;
  ASSERT_TRUE(parallel.configure(geometry, config));
  ASSERT_TRUE(parallel.compute(occupancy, model));
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    EXPECT_FLOAT_EQ(serial.voxels()[i].distance, parallel.voxels()[i].distance);
  }
  config.signed_distance = false;
  ASSERT_TRUE(parallel.configure(geometry, config));
  for (auto &v : occupancy) v.log_odds = 1;
  ASSERT_TRUE(parallel.computeIfChanged(occupancy, model));
  for (const auto &v : static_cast<const esdf::EsdfVolume &>(parallel).voxels()) {
    EXPECT_FLOAT_EQ(0, v.distance);
  }
  for (auto &v : occupancy) v.log_odds = -1;
  ASSERT_TRUE(parallel.computeIfChanged(occupancy, model));
  for (const auto &v : static_cast<const esdf::EsdfVolume &>(parallel).voxels()) {
    EXPECT_FLOAT_EQ(config.max_distance, v.distance);
  }
  config.omp_threads = 0;
  EXPECT_FALSE(parallel.configure(geometry, config));
}

TEST(EsdfLocalTest, SparseUpdatesMatchFullFieldIncludingDistantRemoval) {
  GridGeometry geometry;
  geometry.resolution = 0.03f;
  geometry.size_x = 50; geometry.size_y = 40; geometry.size_z = 34;
  occupancy::LogOddsModel model;
  const int count = geometry.size_x * geometry.size_y * geometry.size_z;
  for (bool signed_distance : {false, true}) {
    for (bool unknown_occupied : {false, true}) {
      esdf::EsdfConfig config;
      config.signed_distance = signed_distance;
      config.unknown_is_occupied = unknown_occupied;
      config.unknown_z_min = 0.15f;
      config.unknown_z_max = 0.45f;
      esdf::EsdfVolume local, full;
      ASSERT_TRUE(local.configure(geometry, config));
      ASSERT_TRUE(full.configure(geometry, config));
      std::vector<OccupancyVoxel> occupancy(count);
      for (auto &v : occupancy) v.log_odds = -1;
      std::mt19937 random(84);
      const auto compare = [&]() {
        ASSERT_TRUE(local.computeIfChanged(occupancy, model));
        ASSERT_TRUE(full.compute(occupancy, model));
        const auto &actual = static_cast<const esdf::EsdfVolume &>(local).voxels();
        const auto &expected = static_cast<const esdf::EsdfVolume &>(full).voxels();
        for (int i = 0; i < count; ++i)
          ASSERT_NEAR(expected[i].distance, actual[i].distance, 1e-6) << i;
      };
      compare();  // all free, then one source, then removal affecting whole map
      occupancy[count / 2].log_odds = 1;
      compare();
      occupancy[count / 2].log_odds = -1;
      compare();
      for (int frame = 0; frame < 15; ++frame) {
        for (int j = 0; j < 12; ++j)
          occupancy[random() % count].log_odds = static_cast<int>(random() % 3) - 1;
        compare();
      }
      for (auto &v : occupancy) v.log_odds = 1;
      compare();  // all occupied, then insert a free cavity and close it
      occupancy[123].log_odds = -1;
      compare();
      occupancy[123].log_odds = 1;
      compare();
      // Mutable access invalidates both intermediate caches.
      local.voxels()[456].distance = -123;
      compare();
      local.clear();
      ASSERT_TRUE(local.configure(geometry, config));
      compare();
    }
  }
}

TEST(EsdfLocalTest, LocalChangeVisitsFewerLinesAndFullModeIsAvailable) {
  GridGeometry geometry;
  geometry.resolution = 0.03f;
  geometry.size_x = geometry.size_y = geometry.size_z = 40;
  occupancy::LogOddsModel model;
  std::vector<OccupancyVoxel> occupancy(40 * 40 * 40);
  // Dense neighboring obstacles bound the influence of a single deletion.
  for (int x = 0; x < 40; ++x) for (int y = 0; y < 40; ++y)
    for (int z = 0; z < 40; ++z)
      occupancy[(x * 40 + y) * 40 + z].log_odds = (x % 4 == 0 && y % 4 == 0 && z % 4 == 0) ? 1 : -1;
  esdf::EsdfConfig config;
  config.unknown_is_occupied = false;
  esdf::EsdfVolume local, full;
  ASSERT_TRUE(local.configure(geometry, config));
  config.local_update = false;
  ASSERT_TRUE(full.configure(geometry, config));
  ASSERT_TRUE(local.computeIfChanged(occupancy, model));
  ASSERT_TRUE(full.computeIfChanged(occupancy, model));
  occupancy[(20 * 40 + 20) * 40 + 20].log_odds = -1;
  ASSERT_TRUE(local.computeIfChanged(occupancy, model));
  ASSERT_TRUE(full.computeIfChanged(occupancy, model));
  EXPECT_GT(local.lastUpdatedLines(), 0u);
  EXPECT_LT(local.lastUpdatedLines(), full.lastUpdatedLines() / 4);
  const auto &actual = static_cast<const esdf::EsdfVolume &>(local).voxels();
  const auto &expected = static_cast<const esdf::EsdfVolume &>(full).voxels();
  for (std::size_t i = 0; i < actual.size(); ++i)
    EXPECT_FLOAT_EQ(expected[i].distance, actual[i].distance);
  ASSERT_TRUE(local.computeIfChanged(occupancy, model));
  EXPECT_EQ(0u, local.lastUpdatedLines());
}

TEST(RayIntegrationTest, SupportFilterRetainsConnectedBoundaryAndClearingRays) {
  GridGeometry geometry;
  geometry.resolution = 0.1f;
  geometry.size_x = geometry.size_y = geometry.size_z = 6;
  std::vector<RayObservation> rays;
  for (const auto &point :
       std::vector<Eigen::Vector3f>{{0.15f, 0.15f, 0.15f},
                                    {0.15f, 0.15f, 0.25f},
                                    {0.45f, 0.45f, 0.45f},
                                    {0.05f, 0.45f, 0.45f}}) {
    rays.push_back({Eigen::Vector3f::Zero(), point, false});
  }
  rays.push_back({Eigen::Vector3f::Zero(), {0.45f, 0.45f, 0.01f}, true});
  filterUnsupportedInteriorObservations(&rays, geometry, {true, 0.15f, 0.01f});
  ASSERT_EQ(4u, rays.size());
  EXPECT_TRUE(rays[1].endpoint.isApprox(Eigen::Vector3f(0.15f, 0.15f, 0.25f)));
  EXPECT_TRUE(rays[2].endpoint.isApprox(Eigen::Vector3f(0.05f, 0.45f, 0.45f)));
  EXPECT_TRUE(rays.back().low_z);
}

TEST(RayIntegrationTest, ConstantTimeAdmissionMatchesVoxelTraversal) {
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  const Eigen::Vector3i size(5, 5, 5);
  std::mt19937 random(7);
  std::uniform_real_distribution<float> position(-1.0f, 2.0f);
  for (int i = 0; i < 1000; ++i) {
    const Eigen::Vector3f from(position(random), position(random),
                               position(random));
    const Eigen::Vector3f to(position(random), position(random),
                             position(random));
    bool visited = false;
    ray_casting::traverseRay(
        origin, 0.2f, size, from, to,
        [&](const Eigen::Vector3i &, bool) { visited = true; });
    EXPECT_EQ(visited,
              ray_casting::intersectsGrid(origin, 0.2f, size, from, to));
  }
}

}  // namespace
}  // namespace plan_env
