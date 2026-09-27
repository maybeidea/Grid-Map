#include <Eigen/Eigen>
#include <pcl/PCLPointCloud2.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
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

struct GridMapSnapshot {
  GridGeometry geometry;
  std::vector<TsdfVoxel> tsdf_voxels;
  std::vector<OccupancyVoxel> occupancy_voxels;
  std::vector<EsdfVoxel> esdf_voxels;
  uint64_t version = 0;
};

class GridMap {
public:
  GridMap();
  ~GridMap();

  GridMap(const GridMap &) = delete;
  GridMap & operator=(const GridMap &) = delete;

  // Called by the ROS subscription callback after converting its message to PCL.
  bool pointCloudCallback(pcl::PCLPointCloud2::ConstPtr cloud);

  std::shared_ptr<const GridMapSnapshot> snapshot() const;

private:
  void mappingLoop();
  void processPointCloud(const pcl::PCLPointCloud2::ConstPtr & cloud);
  void publishSnapshot();

  GridGeometry grid_geometry_;

  std::vector<TsdfVoxel> tsdf_voxels_;
  std::vector<OccupancyVoxel> occupancy_voxels_;

  std::vector<EsdfVoxel> esdf_voxels_;

  std::mutex pending_cloud_mutex_;
  std::condition_variable pending_cloud_cv_;
  pcl::PCLPointCloud2::ConstPtr pending_cloud_;
  bool stopping_ = false;
  std::thread mapping_thread_;

  uint64_t snapshot_version_ = 0;
  std::shared_ptr<const GridMapSnapshot> latest_snapshot_;

  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int x, int y, int z);
  inline int toAddress2d(int x, int y);
  inline void boundIndex(Eigen::Vector3i &id);
  inline void boundIndex(Eigen::Vector2i &id);

};
/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id) {
  return id(0) * grid_geometry_.size_x * grid_geometry_.size_y +
         id(1) * grid_geometry_.size_y + id(2);
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
