#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "plan_env/esdf.hpp"
#include "plan_env/height_map.hpp"
#include "plan_env/occupancy.hpp"
#include "plan_env/ray_casting.hpp"
#include "plan_env/registration.hpp"
#include "plan_env/tsdf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef PLAN_ENV_HAS_BIMAX_MSGS
#include <bimax_msgs/msg/height_map.hpp>
#else
namespace bimax_msgs { namespace msg { struct HeightMap; } }
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
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace plan_env {

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
  double getResolution() const;
  void getRegion(Eigen::Vector3d &origin, Eigen::Vector3d &size) const;
  void boundIndex(Eigen::Vector3i &id) const;
  void boundIndex(Eigen::Vector2i &id) const;

  // Configure the voxel map before starting a task. The task ROI is supplied
  // later through startTaskMapping(), so one GridMap instance can be reused.
  bool configure(float resolution, float map_size_z,
                 float expansion_margin = 0.25f,
                 float expansion_ratio = 0.10f);

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
  TaskRegion taskRegion() const;
  bool pointInTaskRegion(float x, float y, float z) const;

  std::shared_ptr<const EsdfSnapshot> snapshot() const;

  /** Configure the pose gate. Values are per-cloud pose deltas. */
  bool configureMotion(float translation_threshold = 0.03f,
                       float rotation_threshold_rad = 0.02f,
                       uint32_t stationary_frames = 30);
  MappingStatus mappingStatus() const;
  bool readyToMove() const;
  bool buildHeightMapMsg(bimax_msgs::msg::HeightMap &msg,
                         const Eigen::Isometry3d &T_map_from_height);
  void publishHeightMap();

private:
  void mappingLoop();
  // Point-cloud frame entry point: register accepted frames before ray updates.
  void processPointCloudFrame(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);
  struct PreparedCloud {
    std::vector<Eigen::Vector3f> sensor_points;
    Eigen::Isometry3f prior_pose = Eigen::Isometry3f::Identity();
    Eigen::Vector3f camera_position = Eigen::Vector3f::Zero();
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
  void fuseSamePositionFrame();
  void fuseDifferentPositionFrame();
  void updateMapsFromRays();
  void updateOccupancyVoxel(OccupancyVoxel &voxel, bool hit);
  void updateEsdf();
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
  double height_blocked_at_floor_threshold_ = 0.04;
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

  mutable std::mutex motion_mutex_;
  MotionState motion_state_ = MotionState::kWaitingForPose;
  FusionMode last_fusion_mode_ = FusionMode::kSamePosition;
  Eigen::Vector3f last_motion_position_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf last_motion_orientation_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f stationary_anchor_position_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf stationary_anchor_orientation_ = Eigen::Quaternionf::Identity();
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

  inline int toAddress(const Eigen::Vector3i &id) const;
  inline int toAddress(int x, int y, int z) const;
  inline int toAddress2d(int x, int y) const;
};
/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id) const {
  return id(0) * grid_geometry_.size_y * grid_geometry_.size_z +
         id(1) * grid_geometry_.size_z + id(2);
}

inline int GridMap::toAddress(int x, int y, int z) const {
  return x * grid_geometry_.size_y * grid_geometry_.size_z +
         y * grid_geometry_.size_z + z;
}

inline int GridMap::toAddress2d(int x, int y) const {
  return x * grid_geometry_.size_y + y;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id) const {
  Eigen::Vector3i id1;
  id1(0) = std::max(std::min(id(0), grid_geometry_.size_x - 1), 0);
  id1(1) = std::max(std::min(id(1), grid_geometry_.size_y - 1), 0);
  id1(2) = std::max(std::min(id(2), grid_geometry_.size_z - 1), 0);
  id = id1;
}

inline void GridMap::boundIndex(Eigen::Vector2i &id) const {
  Eigen::Vector2i id1;
  id1(0) = std::max(std::min(id(0), grid_geometry_.size_x - 1), 0);
  id1(1) = std::max(std::min(id(1), grid_geometry_.size_y - 1), 0);
  id = id1;
}

}  // namespace plan_env
