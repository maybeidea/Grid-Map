#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "plan_env/ray_casting.hpp"
#include "plan_env/tsdf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

struct OccupancyVoxel {
  float log_odds = 0.0f;

  uint16_t hit_count = 0;
  uint16_t miss_count = 0;
};
struct QualityVoxel {};
struct EsdfVoxel {
  float distance = 100000.0f;
};
enum class OccupancyState { FREE, OCCUPIED, UNKNOWN };

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
  std::vector<EsdfVoxel> esdf_voxels;
  Eigen::Vector3f camera_position = Eigen::Vector3f::Zero();
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
  GridMap();
  ~GridMap();

  GridMap(const GridMap &) = delete;
  GridMap &operator=(const GridMap &) = delete;

  // Library initialization performed by the owning ROS node.
  void initMap(rclcpp::Node::SharedPtr node);

  bool pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);

  // Configure the voxel map before starting a task. The task ROI is supplied
  // later through startTaskMapping(), so one GridMap instance can be reused.
  bool configure(float resolution, float map_size_z,
                 float expansion_margin = 0.25f,
                 float expansion_ratio = 0.10f);

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

private:
  void mappingLoop();
  // Point-cloud frame entry point: transform first, then run ray/map updates.
  void processPointCloudFrame(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);
  struct PreparedCloud {
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector3f camera_position = Eigen::Vector3f::Zero();
    Eigen::Vector3f motion_position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf motion_orientation = Eigen::Quaternionf::Identity();
  };
  bool processPointCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
      PreparedCloud *prepared);
  bool updateMotionState(const PreparedCloud &prepared, FusionMode *mode);
  void resetMotionState();
  // Separate policy entry points. They currently share ray integration until
  // the same-position and cross-position fusion rules are specified.
  void fuseSamePositionFrame();
  void fuseDifferentPositionFrame();
  void updateMapsFromRays();
  void updateOccupancyVoxel(OccupancyVoxel &voxel, bool hit);
  void publishEsdfSnapshot();
  void clearMappingBuffers();

  GridGeometry grid_geometry_;

  tsdf::TsdfVolume tsdf_volume_;
  std::vector<OccupancyVoxel> occupancy_voxels_;

  std::vector<EsdfVoxel> esdf_voxels_;

  mutable std::mutex task_region_mutex_;
  TaskRegion task_region_;
  float resolution_ = 0.05f;
  float map_size_z_ = 0.60f;
  float expansion_margin_ = 0.25f;

  // Points retained by processPointCloud are expressed in map_frame_.
  std::vector<Eigen::Vector3f> cloud_points_;
  Eigen::Vector3f camera_position_ = Eigen::Vector3f::Zero();
  std::string map_frame_ = "map";
  // Base pose is preferred for motion gating so arm/camera motion does not
  // look like chassis motion. If unavailable, the cloud frame pose is used.
  std::string motion_frame_ = "base_footprint";
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
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

  inline void boundIndex(Eigen::Vector3i &id);
  inline void boundIndex(Eigen::Vector2i &id);
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

inline void GridMap::boundIndex(Eigen::Vector3i &id) {
  Eigen::Vector3i id1;
  id1(0) = std::max(std::min(id(0), grid_geometry_.size_x - 1), 0);
  id1(1) = std::max(std::min(id(1), grid_geometry_.size_y - 1), 0);
  id1(2) = std::max(std::min(id(2), grid_geometry_.size_z - 1), 0);
  id = id1;
}

inline void GridMap::boundIndex(Eigen::Vector2i &id) {
  Eigen::Vector2i id1;
  id1(0) = std::max(std::min(id(0), grid_geometry_.size_x - 1), 0);
  id1(1) = std::max(std::min(id(1), grid_geometry_.size_y - 1), 0);
  id = id1;
}
