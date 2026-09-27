#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct GridGeometry {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();

  float resolution = 0.0f;

  int size_x = 0;
  int size_y = 0;
  int size_z = 0;
};
struct TsdfVoxel {
  float distance = 1.0f; // normalized [-1, 1]
  float weight = 0.0f;
};
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

struct EsdfSnapshot {
  GridGeometry geometry;
  std::vector<EsdfVoxel> esdf_voxels;
  uint64_t version = 0;
};

class GridMap {
public:
  GridMap();
  ~GridMap();

  GridMap(const GridMap &) = delete;
  GridMap &operator=(const GridMap &) = delete;

  bool pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);

  std::shared_ptr<const EsdfSnapshot> snapshot() const;

private:
  void mappingLoop();
  void processPointCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);
  void publishEsdfSnapshot();

  GridGeometry grid_geometry_;

  std::vector<TsdfVoxel> tsdf_voxels_;
  std::vector<OccupancyVoxel> occupancy_voxels_;

  std::vector<EsdfVoxel> esdf_voxels_;

  // Points retained by processPointCloud are expressed in map_frame_.
  std::vector<Eigen::Vector3f> cloud_points_;
  Eigen::Vector3f camera_position_ = Eigen::Vector3f::Zero();
  const std::string map_frame_ = "map";
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex pending_cloud_mutex_;
  std::condition_variable pending_cloud_cv_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  bool stopping_ = false;
  std::thread mapping_thread_;

  uint64_t snapshot_version_ = 0;
  std::shared_ptr<const EsdfSnapshot> latest_esdf_snapshot_;

  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int x, int y, int z);
  inline int toAddress2d(int x, int y);
  inline void boundIndex(Eigen::Vector3i &id);
  inline void boundIndex(Eigen::Vector2i &id);
};
/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id) {
  return id(0) * grid_geometry_.size_y * grid_geometry_.size_z +
         id(1) * grid_geometry_.size_z + id(2);
}

inline int GridMap::toAddress(int x, int y, int z) {
  return x * grid_geometry_.size_y * grid_geometry_.size_z +
         y * grid_geometry_.size_z + z;
}

inline int GridMap::toAddress2d(int x, int y) {
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
