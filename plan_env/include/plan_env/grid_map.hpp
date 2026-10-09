#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "plan_env/esdf.hpp"
#include "plan_env/height_map.hpp"
#include "plan_env/occupancy.hpp"
#include "plan_env/registration.hpp"
#include "plan_env/tsdf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef PLAN_ENV_HAS_BIMAX_MSGS
#include <bimax_msgs/msg/height_map.hpp>
#else
namespace bimax_msgs {
namespace msg {
struct HeightMap;
}
}  // namespace bimax_msgs
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif
#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace plan_env {

// Compatibility placeholder for clients of the original map interface.
struct QualityVoxel {};

/** Mapping gate driven by consecutive timestamped TF poses. */
enum class MotionState {
  kWaitingForPose,
  kMoving,
  kStationaryCollecting,
  kReadyToMove,
};

/** The fusion policy is deliberately selected before the map update. */
enum class FusionMode {
  kSamePosition,
  kDifferentPosition,
};

struct MappingStatus {
  MotionState motion_state = MotionState::kWaitingForPose;
  FusionMode last_fusion_mode = FusionMode::kSamePosition;
  uint32_t stationary_frame_count = 0;
  uint32_t stationary_frames_required = 30;
  uint64_t position_index = 0;
  uint64_t same_position_fused_frames = 0;
  uint64_t different_position_fused_frames = 0;
  bool ready_to_move = false;
};

struct EsdfSnapshot {
  GridGeometry geometry;
  std::vector<TsdfVoxel> tsdf_voxels;
  std::vector<OccupancyVoxel> occupancy_voxels;
  std::vector<float> occupancy_probabilities;
  std::vector<OccupancyState> occupancy_states;
  std::vector<EsdfVoxel> esdf_voxels;
  Eigen::Vector3f camera_position = Eigen::Vector3f::Zero();
  bool tsdf_enabled = false;
  // The same expanded XY rectangle is used as the task ROI and map window.
  std::vector<Eigen::Vector2f> task_roi_polygon;
  bool task_mapping_active = false;
  MappingStatus mapping_status;
  uint64_t version = 0;
};

struct TaskRegion {
  bool valid = false;
  Eigen::Vector2f min = Eigen::Vector2f::Zero();
  Eigen::Vector2f max = Eigen::Vector2f::Zero();
  float z_min = 0.0f;
  float z_max = 0.0f;
  float expansion_margin = 0.0f;
  float expansion_ratio = 0.0f;
  std::vector<Eigen::Vector2d> source_polygon;
  std::vector<Eigen::Vector2f> expanded_polygon;

  bool contains(float x, float y, float z) const {
    return valid && x >= min.x() && x <= max.x() && y >= min.y() &&
           y <= max.y() && z >= z_min && z <= z_max;
  }
};

class GridMap {
public:
  enum { INVALID_IDX = -10000 };
  GridMap();
  ~GridMap();

  GridMap(const GridMap &) = delete;
  GridMap &operator=(const GridMap &) = delete;

  // Library initialization performed by the owning ROS node.
  void initMap(rclcpp::Node::SharedPtr node);

  bool pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);

  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) const;
  bool isInMap(const Eigen::Vector3i &id) const;
  bool isInMap(const Eigen::Vector2i &id) const;
  bool isInMap(const Eigen::Vector3d &pos) const;
  bool isInMap(const Eigen::Vector2d &pos) const;
  Eigen::Vector2i pos2dToIndex(const Eigen::Vector2d &pos) const;
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) const;
  Eigen::Vector2d index2dToPos(const Eigen::Vector2i &id) const;
  double getResolution() const;
  Eigen::Vector3d getOrigin() const;
  void getVoxelNum(Eigen::Vector3i &voxel_num) const;
  void getRegion(Eigen::Vector3d &origin, Eigen::Vector3d &size) const;
  void boundIndex(Eigen::Vector3i &id) const;
  void boundIndex(Eigen::Vector2i &id) const;

  // Configure the voxel map before starting a task. The task ROI is supplied
  // later through startTaskMapping(), so one GridMap instance can be reused.
  bool configure(float resolution, float map_size_z,
                 float expansion_margin = 0.25f, float expansion_ratio = 0.10f);

  // Configure the sensor update increments and occupancy decision thresholds.
  bool configureOccupancy(const occupancy::LogOddsConfig &config);
  // Configure occupancy-to-ESDF conversion before starting a task.
  bool configureEsdf(const esdf::EsdfConfig &config);
  // Configure optional TSDF fusion before starting a task. Occupancy remains
  // the collision map and is always updated when a task is active.
  bool configureTsdf(bool enabled, float truncation_distance = 0.05f);
  // Configure the input point-cloud downsampling leaf size.
  bool configurePointCloudFilter(float leaf_size);
  bool configureRegistration(const registration::Config &config);

  // Record a task polygon and derive one expanded rectangular ROI. This same
  // rectangle becomes both the map window and the ESDF/task ROI.
  bool startTaskMapping(const std::vector<Eigen::Vector2d> &polygon_xy,
                        std::string *message_out = nullptr);
  void stopTaskMapping();
  bool isTaskMappingActive() const;
  bool isMappingReadyForPlanning() const;
  TaskRegion taskRegion() const;
  bool pointInTaskRegion(float x, float y, float z) const;

  std::shared_ptr<const EsdfSnapshot> snapshot() const;
  double getDistance(const Eigen::Vector3d &pos) const;
  double getDistance(const Eigen::Vector3i &id) const;
  double getPreciseDistance(const Eigen::Vector3d &pos) const;
  int getOccupancy(const Eigen::Vector3d &pos) const;
  int getOccupancy(const Eigen::Vector3i &id) const;
  bool isUnknown(const Eigen::Vector3i &id) const;
  bool isUnknown(const Eigen::Vector3d &pos) const;
  bool isKnownFree(const Eigen::Vector3i &id) const;
  bool isKnownOccupied(const Eigen::Vector3i &id) const;
  bool isKnownOccupied(const Eigen::Vector3d &pos) const;

  void freezeOccupancyAndEsdf();
  void captureEsdfFreezing();
  void clearEsdfFreezing();
  bool hasEsdfFreezing() const;
  void setEsdfFreezingQueryActive(bool active);
  bool esdfFreezingQueryActive() const;
  bool buildEsdfFreezingSliceCloud(
      sensor_msgs::msg::PointCloud2 &cloud_msg) const;
  bool buildEsdfFreezingVolumeCloud(
      sensor_msgs::msg::PointCloud2 &cloud_msg) const;
  bool saveEsdfSnapshot(const std::string &output_dir, const std::string &tag,
                        std::string *saved_basename_out = nullptr,
                        bool save_local3d = true, bool save_slice = true) const;

  /** Configure the pose gate. Values are per-cloud pose deltas. */
  bool configureMotion(float translation_threshold = 0.03f,
                       float rotation_threshold_rad = 0.02f,
                       uint32_t stationary_frames = 30);
  MappingStatus mappingStatus() const;
  bool readyToMove() const;
  bool buildHeightMapMsg(bimax_msgs::msg::HeightMap &msg,
                         const Eigen::Isometry3d &T_map_from_height);
  bool buildHeightMapMsg(bimax_msgs::msg::HeightMap &msg);
  void publishHeightMap();

  // Legacy planner adapters; no A* storage or odometry timeout is maintained.
  void resetAstarBuffer();
  bool isInTaskPessimismPolygon(double x, double y) const;
  bool odomValid() const;
  bool getOdomDepthTimeout() const;

private:
  // Select one immutable snapshot for the entire query.
  std::shared_ptr<const EsdfSnapshot> querySnapshot() const;
  void mappingLoop();
  // Point-cloud frame entry point: register accepted frames before ray updates.
  // Returns true only when this frame completes the configured fusion quota
  // for the current stationary position.
  bool processPointCloudFrame(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);
  struct PreparedCloud {
    std::vector<Eigen::Vector3f> sensor_points;
    Eigen::Isometry3f prior_pose = Eigen::Isometry3f::Identity();
    Eigen::Vector3f motion_position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf motion_orientation = Eigen::Quaternionf::Identity();
  };
  bool processPointCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
      PreparedCloud *prepared);
  bool registerDifferentPosition(const PreparedCloud &prepared,
                                 Eigen::Isometry3f *pose);
  bool updateMotionState(const PreparedCloud &prepared, FusionMode *mode);
  void resetMotionState();
  // Separate policy entry points keep temporal and cross-position fusion
  // policies independent of the occupancy integrator.
  bool fuseSamePositionFrame();
  bool fuseDifferentPositionFrame();
  bool updateMapsFromRays();
  bool recordSuccessfulFusion();
  void publishEsdfSnapshot();
  void clearMappingBuffers();

  GridGeometry grid_geometry_;

  tsdf::TsdfVolume tsdf_volume_;
  bool tsdf_enabled_ = false;
  float tsdf_truncation_distance_ = 0.05f;
  occupancy::LogOddsModel occupancy_model_;
  esdf::EsdfConfig esdf_config_;
  std::vector<OccupancyVoxel> occupancy_voxels_;
  esdf::EsdfVolume esdf_volume_;
  registration::PointCloudRegistrationBackend registration_backend_;
  std::vector<Eigen::Vector3f> registration_target_points_;

  mutable std::mutex task_region_mutex_;
  TaskRegion task_region_;
  float resolution_ = 0.05f;
  float map_size_z_ = 0.60f;
  float expansion_margin_ = 0.25f;
  float cloud_leaf_size_ = 0.005f;
  bool height_map_enable_ = true;
  std::string height_map_topic_ = "/complex_area_plan/height";
  std::string height_map_frame_ = "base_footprint";
  double height_band_z_max_ = 0.50;
  double height_completeness_z_max_ = 0.20;
  double height_floor_tol_ = 0.02;
  double height_vision_limit_ = 0.10;
  double height_z_tcp_min_ = 0.07;
  double height_ceil_threshold_open_ = 0.50;

  // Points retained by processPointCloud are expressed in map_frame_.
  std::vector<Eigen::Vector3f> cloud_points_;
  Eigen::Vector3f camera_position_ = Eigen::Vector3f::Zero();
  std::string map_frame_ = "map";
  // Base pose is preferred for motion gating so arm/camera motion does not
  // look like chassis motion. If unavailable, the cloud frame pose is used.
  std::string motion_frame_ = "base_footprint";
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef PLAN_ENV_HAS_BIMAX_MSGS
  rclcpp::Publisher<bimax_msgs::msg::HeightMap>::SharedPtr height_map_pub_;
#endif
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex pending_cloud_mutex_;
  std::condition_variable pending_cloud_cv_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  bool stopping_ = false;
  std::thread mapping_thread_;

  uint64_t snapshot_version_ = 0;
  mutable std::mutex snapshot_mutex_;
  std::shared_ptr<const EsdfSnapshot> latest_esdf_snapshot_;
  std::shared_ptr<const EsdfSnapshot> frozen_esdf_snapshot_;
  bool esdf_freezing_query_active_ = false;

  mutable std::mutex motion_mutex_;
  MotionState motion_state_ = MotionState::kWaitingForPose;
  FusionMode last_fusion_mode_ = FusionMode::kSamePosition;
  Eigen::Vector3f last_motion_position_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf last_motion_orientation_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f stationary_anchor_position_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf stationary_anchor_orientation_ =
      Eigen::Quaternionf::Identity();
  bool has_last_motion_pose_ = false;
  bool has_stationary_anchor_ = false;
  bool awaiting_new_position_ = false;
  uint32_t stationary_frame_count_ = 0;
  uint32_t stationary_frames_required_ = 30;
  uint64_t position_index_ = 0;
  uint64_t same_position_fused_frames_ = 0;
  uint64_t different_position_fused_frames_ = 0;
  float motion_translation_threshold_ = 0.03f;
  float motion_rotation_threshold_rad_ = 0.02f;

  int toAddress(const Eigen::Vector3i &id) const {
    return toAddress(id.x(), id.y(), id.z());
  }

  int toAddress(int x, int y, int z) const {
    return (x * grid_geometry_.size_y + y) * grid_geometry_.size_z + z;
  }
};

}  // namespace plan_env
