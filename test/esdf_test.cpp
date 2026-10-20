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
