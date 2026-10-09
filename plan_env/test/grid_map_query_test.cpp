// Both include paths must be safe in the same translation unit.
#include "plan_env/grid_map.h"
#include "plan_env/grid_map.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace plan_env {

// Inject timestamped TF without relying on DDS scheduling. The production
// cloud preparation, gate, registration, ray integrator and publisher all run.
struct GridMapTestAccess {
  static sensor_msgs::msg::PointCloud2::SharedPtr cloud(
      const std::vector<Eigen::Vector3f> &points) {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    for (const auto &p : points) pcl_cloud.emplace_back(p.x(), p.y(), p.z());
    pcl::toROSMsg(pcl_cloud, *msg);
    msg->header.frame_id = "map";
    return msg;
  }

  static uint64_t generation(GridMap &map) {
    std::lock_guard<std::recursive_mutex> lock(map.task_region_mutex_);
    return map.task_generation_;
  }

  static bool queued(GridMap &map,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                     uint64_t generation) {
    return map.processQueuedCloud(msg, generation);
  }

  static bool frame(GridMap &map, float x = 0.0f, float yaw = 0.0f,
                    const std::vector<Eigen::Vector3f> &points = {{0.625f, 0.125f, 0.375f}}) {
    std::lock_guard<std::recursive_mutex> lock(map.task_region_mutex_);
    static std::atomic<int> seconds{1};
    const auto stamp = rclcpp::Time(seconds.fetch_add(1), 0, RCL_ROS_TIME);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_footprint";
    tf.header.stamp = stamp;
    tf.transform.translation.x = x;
    tf.transform.rotation.z = std::sin(yaw * 0.5f);
    tf.transform.rotation.w = std::cos(yaw * 0.5f);
    map.tf_buffer_->setTransform(tf, "test", false);
    auto msg = cloud(points);
    msg->header.stamp = stamp;
    return queued(map, msg, map.task_generation_);
  }
};

namespace {

class GridMapQueryTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    map_ = std::make_unique<GridMap>();
    ASSERT_TRUE(map_->configure(0.25f, 1.0f, 0.0f));
    ASSERT_TRUE(map_->configureMotion(0.03f, 0.02f, 1));
    start();
  }

  void start() {
    ASSERT_TRUE(map_->startTaskMapping({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}));
  }

  bool frame(float x = 0.0f, float yaw = 0.0f,
             const std::vector<Eigen::Vector3f> &points = {{0.625f, 0.125f, 0.375f}}) {
    return GridMapTestAccess::frame(*map_, x, yaw, points);
  }

  std::unique_ptr<GridMap> map_;
};

TEST_F(GridMapQueryTest, resolutionIsAvailableBeforeStartAndAfterStop) {
  GridMap other;
  EXPECT_NEAR(0.05, other.getResolution(), 1e-7);
  ASSERT_TRUE(other.configure(0.125f, 1.0f));
  EXPECT_DOUBLE_EQ(0.125, other.getResolution());
  ASSERT_TRUE(other.startTaskMapping({{0, 0}, {1, 0}, {0, 1}}));
  other.stopTaskMapping();
  EXPECT_DOUBLE_EQ(0.125, other.getResolution());
  EXPECT_FALSE(other.isMappingReadyForPlanning());
}

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

TEST_F(GridMapQueryTest, stopAndRestartInvalidateLiveFrozenAndDequeuedFrames) {
  EXPECT_FALSE(frame());
  ASSERT_TRUE(frame());
  const Eigen::Vector3d occupied(0.625, 0.125, 0.375);
  ASSERT_EQ(1, map_->getOccupancy(occupied));
  const auto old_snapshot = map_->snapshot();
  const auto old_generation = GridMapTestAccess::generation(*map_);
  map_->captureEsdfFreezing();
  map_->setEsdfFreezingQueryActive(true);
  ASSERT_TRUE(map_->hasEsdfFreezing());
  sensor_msgs::msg::PointCloud2 cloud;
  ASSERT_TRUE(map_->buildEsdfFreezingSliceCloud(cloud));
  EXPECT_EQ(64u, cloud.width);
  ASSERT_TRUE(map_->buildEsdfFreezingVolumeCloud(cloud));
  EXPECT_EQ(1u, cloud.width);

  map_->stopTaskMapping();
  EXPECT_FALSE(map_->isTaskMappingActive());
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(map_->esdfFreezingQueryActive());
  EXPECT_FALSE(map_->buildEsdfFreezingSliceCloud(cloud));
  EXPECT_EQ(-1, map_->getOccupancy(occupied));
  ASSERT_TRUE(map_->configure(0.5f, 1.0f, 0.0f));
  ASSERT_TRUE(map_->startTaskMapping({{10, 10}, {11, 10}, {11, 11}, {10, 11}}));
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  map_->captureEsdfFreezing();
  map_->setEsdfFreezingQueryActive(true);
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(map_->esdfFreezingQueryActive());
  EXPECT_TRUE(map_->snapshot()->esdf_voxels.empty());
  auto delayed = GridMapTestAccess::cloud({{0.625f, 0.125f, 0.375f}});
  EXPECT_FALSE(GridMapTestAccess::queued(*map_, delayed, old_generation));
  EXPECT_EQ(0u, map_->mappingStatus().stationary_frame_count);
  // Readers already holding an immutable snapshot remain memory-safe.
  EXPECT_EQ(256u, old_snapshot->esdf_voxels.size());
}

TEST_F(GridMapQueryTest, preciseDistanceInterpolatesVoxelCenters) {
  EXPECT_FALSE(frame());
  ASSERT_TRUE(frame());
  const auto snapshot = map_->snapshot();
  double expected = 0.0;
  for (int x = 5; x <= 6; ++x) {
    for (int y = 3; y <= 4; ++y) {
      for (int z = 0; z <= 1; ++z) {
        expected += snapshot->esdf_voxels[(x * 8 + y) * 4 + z].distance / 8.0;
      }
    }
  }
  EXPECT_DOUBLE_EQ(expected, map_->getPreciseDistance(Eigen::Vector3d(0.5, 0.0, 0.25)));
}

TEST_F(GridMapQueryTest, translationAndRotationResumeAtThreeSuccessiveStops) {
  ASSERT_TRUE(map_->configureMotion(0.03f, 0.02f, 2));
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  map_->freezeOccupancyAndEsdf();
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(frame());
  EXPECT_FALSE(frame());
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  ASSERT_TRUE(frame());
  EXPECT_TRUE(map_->isMappingReadyForPlanning());
  const auto first_version = map_->snapshot()->version;
  EXPECT_FALSE(frame(0.005f));  // jitter after quota must not fuse again
  EXPECT_EQ(first_version, map_->snapshot()->version);

  EXPECT_FALSE(frame(0.5f));
  EXPECT_EQ(MotionState::kMoving, map_->mappingStatus().motion_state);
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  map_->captureEsdfFreezing();
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(frame(0.5f));
  EXPECT_EQ(FusionMode::kDifferentPosition, map_->mappingStatus().last_fusion_mode);
  ASSERT_TRUE(frame(0.5f));
  EXPECT_TRUE(map_->isMappingReadyForPlanning());
  EXPECT_EQ(2u, map_->mappingStatus().position_index);

  EXPECT_FALSE(frame(0.5f, 0.3f));
  EXPECT_FALSE(frame(0.5f, 0.3f));
  ASSERT_TRUE(frame(0.5f, 0.3f));
  EXPECT_EQ(3u, map_->mappingStatus().position_index);
  EXPECT_EQ(2u, map_->mappingStatus().different_position_fused_frames);
  EXPECT_TRUE(map_->readyToMove());
}

TEST_F(GridMapQueryTest, cumulativeDriftInterruptsQuotaAndResetsNewStop) {
  ASSERT_TRUE(map_->configureMotion(0.03f, 0.02f, 3));
  EXPECT_FALSE(frame());
  EXPECT_FALSE(frame());
  EXPECT_FALSE(frame(0.02f));
  EXPECT_EQ(2u, map_->mappingStatus().stationary_frame_count);
  EXPECT_FALSE(frame(0.04f));  // small adjacent delta, large anchor delta
  EXPECT_EQ(MotionState::kMoving, map_->mappingStatus().motion_state);
  EXPECT_EQ(0u, map_->mappingStatus().stationary_frame_count);
  EXPECT_FALSE(frame(0.04f));
  EXPECT_FALSE(frame(0.04f));
  ASSERT_TRUE(frame(0.04f));
  EXPECT_EQ(2u, map_->mappingStatus().position_index);
}

TEST_F(GridMapQueryTest, failedRegistrationCannotBecomeSamePositionFusion) {
  map_->stopTaskMapping();
  registration::Config cfg;
  cfg.min_target_points = 1;
  cfg.max_correspondence_distance = 0.01f;
  ASSERT_TRUE(map_->configureRegistration(cfg));
  start();
  const std::vector<Eigen::Vector3f> first{{0.6f, 0.1f, 0.3f}, {0.7f, 0.2f, 0.4f},
                                         {0.5f, 0.3f, 0.6f}, {0.8f, 0.4f, 0.5f}};
  EXPECT_FALSE(frame(0, 0, first));
  ASSERT_TRUE(frame(0, 0, first));
  const auto version = map_->snapshot()->version;
  EXPECT_FALSE(frame(0.5f, 0, first));
  const std::vector<Eigen::Vector3f> far{{-0.6f, -0.6f, 0.3f}, {-0.7f, -0.5f, 0.4f},
                                       {-0.5f, -0.4f, 0.6f}, {-0.8f, -0.3f, 0.5f}};
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(frame(0.5f, 0, far));
    EXPECT_EQ(0u, map_->mappingStatus().stationary_frame_count);
    EXPECT_EQ(FusionMode::kDifferentPosition, map_->mappingStatus().last_fusion_mode);
    EXPECT_EQ(version, map_->snapshot()->version);
  }
  ASSERT_TRUE(frame(0.5f, 0, first));  // valid overlap retries and recovers
  EXPECT_TRUE(map_->isMappingReadyForPlanning());
}

TEST_F(GridMapQueryTest, acceptedRegistrationIsRetainedForTheRestOfTheStop) {
  map_->stopTaskMapping();
  registration::Config cfg;
  cfg.min_target_points = 1;
  cfg.max_correspondence_distance = 0.2f;
  ASSERT_TRUE(map_->configureRegistration(cfg));
  ASSERT_TRUE(map_->configureMotion(0.03f, 0.02f, 2));
  start();
  const std::vector<Eigen::Vector3f> reference{{0.6f, 0.1f, 0.3f}, {0.7f, 0.2f, 0.4f},
                                            {0.5f, 0.3f, 0.6f}, {0.8f, 0.4f, 0.5f}};
  EXPECT_FALSE(frame(0, 0, reference));
  EXPECT_FALSE(frame(0, 0, reference));
  ASSERT_TRUE(frame(0, 0, reference));
  auto shifted = reference;
  for (auto &p : shifted) p.x() += 0.05f;
  EXPECT_FALSE(frame(0.5f, 0, shifted));
  EXPECT_FALSE(frame(0.5f, 0, shifted));  // first accepted cross-position frame
  EXPECT_EQ(1u, map_->mappingStatus().stationary_frame_count);
  ASSERT_TRUE(frame(0.5f, 0, shifted));   // same-position frame must retain correction
  EXPECT_NEAR(-0.05f, map_->snapshot()->camera_position.x(), 1e-3f);
  EXPECT_EQ(1u, map_->mappingStatus().different_position_fused_frames);
}

TEST_F(GridMapQueryTest, accumulatedRotationResetsTheStationAndRecovers) {
  ASSERT_TRUE(map_->configureMotion(0.03f, 0.02f, 3));
  EXPECT_FALSE(frame());
  EXPECT_FALSE(frame());
  EXPECT_FALSE(frame(0, 0.015f));
  EXPECT_FALSE(frame(0, 0.030f));
  EXPECT_EQ(MotionState::kMoving, map_->mappingStatus().motion_state);
  EXPECT_EQ(0u, map_->mappingStatus().stationary_frame_count);
  EXPECT_FALSE(frame(0, 0.030f));
  EXPECT_FALSE(frame(0, 0.030f));
  ASSERT_TRUE(frame(0, 0.030f));
  EXPECT_TRUE(map_->readyToMove());
}

TEST_F(GridMapQueryTest, outsideRoiDoesNotAdvanceFusionQuota) {
  EXPECT_FALSE(frame());
  EXPECT_FALSE(frame(0, 0, {{3.0f, 0.1f, 0.3f}}));
  EXPECT_EQ(0u, map_->mappingStatus().stationary_frame_count);
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  EXPECT_TRUE(frame());
}

TEST_F(GridMapQueryTest, hardFreezeStopsUpdatesUntilReleased) {
  EXPECT_FALSE(frame());
  ASSERT_TRUE(frame());
  map_->freezeOccupancyAndEsdf();
  ASSERT_TRUE(map_->esdfFreezingQueryActive());
  const auto version = map_->snapshot()->version;
  EXPECT_FALSE(frame(0.5f));
  EXPECT_FALSE(frame(0.5f));
  EXPECT_EQ(version, map_->snapshot()->version);
  map_->clearEsdfFreezing();
  EXPECT_FALSE(frame(0.5f));
  EXPECT_TRUE(frame(0.5f));
  EXPECT_GT(map_->snapshot()->version, version);
}

TEST_F(GridMapQueryTest, legacyGroundFilterDropsOrClearsWithoutCreatingHit) {
  for (bool clear : {false, true}) {
    map_->stopTaskMapping();
    rclcpp::NodeOptions options;
    options.append_parameter_override("grid_map.resolution", 0.25);
    options.append_parameter_override("grid_map.map_size_z", 1.0);
    options.append_parameter_override("grid_map.cloud_min_map_z", 0.2);
    options.append_parameter_override("grid_map.cloud_low_z_clear_rays", clear);
    options.append_parameter_override("grid_map.stationary_frames", 1);
    auto node = std::make_shared<rclcpp::Node>(clear ? "ground_clear" : "ground_drop", options);
    map_->initMap(node);
    start();
    EXPECT_FALSE(frame(0, 0, {{0.625f, 0.125f, 0.1f}}));
    EXPECT_EQ(clear, frame(0, 0, {{0.625f, 0.125f, 0.1f}}));
    EXPECT_EQ(clear ? 0 : -1, map_->getOccupancy(Eigen::Vector3d(0.625, 0.125, 0.1)));
    EXPECT_EQ(clear ? 1u : 0u, map_->mappingStatus().stationary_frame_count);
  }
}

TEST_F(GridMapQueryTest, unknownHeightBandOnlyLimitsUnknownSources) {
  GridGeometry geometry;
  geometry.resolution = 0.25f;
  geometry.size_x = geometry.size_y = 1;
  geometry.size_z = 4;
  esdf::EsdfConfig cfg;
  cfg.unknown_z_min = 0.0f;
  cfg.unknown_z_max = 0.25f;
  esdf::EsdfVolume volume;
  ASSERT_TRUE(volume.configure(geometry, cfg));
  std::vector<OccupancyVoxel> voxels(4);
  ASSERT_TRUE(volume.compute(voxels));
  EXPECT_LT(volume.voxels()[0].distance, 0);
  EXPECT_LT(volume.voxels()[1].distance, 0);  // inclusive voxel-layer boundary
  EXPECT_GT(volume.voxels()[2].distance, 0);
  occupancy::LogOddsModel model;
  model.update(voxels[3], true);
  ASSERT_TRUE(volume.compute(voxels, model));
  EXPECT_LT(volume.voxels()[3].distance, 0);  // known obstacle above ROI band
  cfg.unknown_is_occupied = false;
  ASSERT_TRUE(volume.configure(geometry, cfg));
  ASSERT_TRUE(volume.compute(voxels, model));
  EXPECT_GT(volume.voxels()[0].distance, 0);
  EXPECT_LT(volume.voxels()[3].distance, 0);
}

TEST_F(GridMapQueryTest, legacyUnknownModeParametersSelectOffTaskBandAndAll) {
  for (const std::string mode : {"off", "task_roi", "all"}) {
    map_->stopTaskMapping();
    rclcpp::NodeOptions options;
    options.append_parameter_override("grid_map.resolution", 0.25);
    options.append_parameter_override("grid_map.map_size_z", 1.0);
    options.append_parameter_override("grid_map.esdf_unknown_mode", mode);
    options.append_parameter_override("grid_map.ground_height", 0.25);
    options.append_parameter_override("grid_map.esdf_roi_z_max", 0.50);
    options.append_parameter_override("grid_map.stationary_frames", 1);
    auto node = std::make_shared<rclcpp::Node>("unknown_" + mode, options);
    map_->initMap(node);
    start();
    EXPECT_FALSE(frame());
    ASSERT_TRUE(frame());
    const double below = map_->getDistance(Eigen::Vector3d(-0.8, -0.8, 0.125));
    const double inside = map_->getDistance(Eigen::Vector3d(-0.8, -0.8, 0.375));
    const double above = map_->getDistance(Eigen::Vector3d(-0.8, -0.8, 0.875));
    EXPECT_EQ(mode == "all", below < 0.0) << mode;
    EXPECT_EQ(mode != "off", inside < 0.0) << mode;
    EXPECT_EQ(mode == "all", above < 0.0) << mode;
  }
}

TEST_F(GridMapQueryTest, directTaskReplacementAlsoDiscardsFrozenData) {
  EXPECT_FALSE(frame());
  ASSERT_TRUE(frame());
  map_->freezeOccupancyAndEsdf();
  ASSERT_TRUE(map_->hasEsdfFreezing());
  start();  // replace without a separate stop
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  EXPECT_TRUE(map_->snapshot()->esdf_voxels.empty());
  EXPECT_FALSE(frame());
  EXPECT_TRUE(frame());
}

TEST_F(GridMapQueryTest, queuedCloudsAndTaskSwitchesAreSerialized) {
  auto msg = GridMapTestAccess::cloud({{0.625f, 0.125f, 0.375f}});
  std::atomic<bool> done{false};
  std::thread producer([&] {
    while (!done.load()) {
      map_->pointCloudCallback(msg);
      std::this_thread::yield();
    }
  });
  for (int i = 0; i < 20; ++i) {
    map_->stopTaskMapping();
    start();
    map_->captureEsdfFreezing();
    (void)map_->getDistance(Eigen::Vector3d(0.625, 0.125, 0.375));
  }
  done.store(true);
  producer.join();
  map_->stopTaskMapping();
  EXPECT_FALSE(map_->pointCloudCallback(msg));
  EXPECT_FALSE(map_->hasEsdfFreezing());
  EXPECT_FALSE(map_->isMappingReadyForPlanning());
  EXPECT_TRUE(map_->snapshot()->esdf_voxels.empty());
}

}  // namespace
}  // namespace plan_env
