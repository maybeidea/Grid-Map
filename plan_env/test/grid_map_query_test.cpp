// Both include paths must be safe in the same translation unit.
#include "plan_env/grid_map.h"
#include "plan_env/grid_map.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace plan_env {
namespace {

class GridMapQueryTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    map_ = std::make_unique<GridMap>();
    ASSERT_TRUE(map_->configure(0.25f, 1.0f, 0.0f));
    ASSERT_TRUE(map_->configureMotion(0.03f, 0.02f, 1));
    ASSERT_TRUE(map_->startTaskMapping({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}));
  }

  bool processCloud(bool with_point) {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    if (with_point) {
      pcl::PointCloud<pcl::PointXYZ> points;
      points.push_back(pcl::PointXYZ(0.625f, 0.125f, 0.375f));
      pcl::toROSMsg(points, *cloud);
    }
    cloud->header.frame_id = "map";
    const auto version = map_->snapshot()->version;
    if (!map_->pointCloudCallback(cloud)) {
      return false;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      if (map_->snapshot()->version > version) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  std::unique_ptr<GridMap> map_;
};

TEST_F(GridMapQueryTest, voxelCentersRoundTripInTwoAndThreeDimensions) {
  const Eigen::Vector3i index(6, 4, 1);
  Eigen::Vector3d position;
  map_->indexToPos(index, position);
  EXPECT_TRUE(position.isApprox(Eigen::Vector3d(0.625, 0.125, 0.375)));
  Eigen::Vector3i round_trip;
  map_->posToIndex(position, round_trip);
  EXPECT_EQ(index, round_trip);
  const Eigen::Vector2i index_xy = index.head<2>();
  EXPECT_EQ(index_xy, map_->pos2dToIndex(map_->index2dToPos(index_xy)));
}

TEST_F(GridMapQueryTest, frozenQueriesKeepTheirGeometryWhenTheLiveTaskChanges) {
  EXPECT_FALSE(processCloud(true));  // Establish the first pose.
  ASSERT_TRUE(processCloud(true));    // Complete one-fusion position.
  const Eigen::Vector3d occupied(0.625, 0.125, 0.375);
  const Eigen::Vector3i index(6, 4, 1);
  ASSERT_EQ(1, map_->getOccupancy(occupied));
  const double distance = map_->getDistance(occupied);
  ASSERT_LT(distance, 0.0);
  const Eigen::Vector3d interpolation_point(0.5, 0.0, 0.25);
  const double interpolated = map_->getPreciseDistance(interpolation_point);

  map_->captureEsdfFreezing();
  map_->setEsdfFreezingQueryActive(true);
  map_->stopTaskMapping();
  ASSERT_TRUE(map_->configure(0.5f, 1.0f, 0.0f));
  ASSERT_TRUE(map_->startTaskMapping({{10, 10}, {11, 10}, {11, 11}, {10, 11}}));
  EXPECT_FALSE(processCloud(false));  // No fusion, no position-complete update.

  EXPECT_EQ(1, map_->getOccupancy(index));
  EXPECT_EQ(1, map_->getOccupancy(occupied));
  EXPECT_DOUBLE_EQ(distance, map_->getDistance(index));
  EXPECT_DOUBLE_EQ(distance, map_->getDistance(occupied));
  EXPECT_DOUBLE_EQ(interpolated, map_->getPreciseDistance(interpolation_point));

  sensor_msgs::msg::PointCloud2 cloud;
  ASSERT_TRUE(map_->buildEsdfFreezingSliceCloud(cloud));
  EXPECT_EQ(64u, cloud.width);
  ASSERT_TRUE(map_->buildEsdfFreezingVolumeCloud(cloud));
  EXPECT_EQ(1u, cloud.width);

  map_->clearEsdfFreezing();
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(map_->esdfFreezingQueryActive());
  // The live ESDF snapshot is also held until the new position completes its
  // fusion quota, so clearing the frozen override reveals the last published
  // snapshot rather than a partially updated map.
  EXPECT_EQ(1, map_->getOccupancy(occupied));
  EXPECT_DOUBLE_EQ(distance, map_->getDistance(occupied));
}

TEST_F(GridMapQueryTest, preciseDistanceInterpolatesVoxelCenters) {
  EXPECT_FALSE(processCloud(true));
  ASSERT_TRUE(processCloud(true));
  const auto snapshot = map_->snapshot();
  double expected = 0.0;
  for (int x = 5; x <= 6; ++x) {
    for (int y = 3; y <= 4; ++y) {
      for (int z = 0; z <= 1; ++z) {
        const auto address = (x * 8 + y) * 4 + z;
        expected += snapshot->esdf_voxels[address].distance / 8.0;
      }
    }
  }
  EXPECT_DOUBLE_EQ(expected,
                   map_->getPreciseDistance(Eigen::Vector3d(0.5, 0.0, 0.25)));
}

}  // namespace
}  // namespace plan_env
