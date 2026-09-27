#include "grid_map.hpp"

#include <Eigen/Eigen>
#include <algorithm>
#include <cerrno>
#include <atomic>
#include <cstring>
// #include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <sstream>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
// #include <tf2/exceptions.h>

using std::max;
using std::min;
using std::string;
using std::vector;

namespace {

constexpr double kDefaultDistance = 10000.0;

bool shouldClearOnRayStaleOccupied(const int hit_count, const int miss_count, const int min_miss)
{
  return hit_count == 0 && miss_count >= min_miss;
}

bool pointInPolygonXy(double x, double y, const std::vector<Eigen::Vector2d> & poly)
{
  const int n = static_cast<int>(poly.size());
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double yi = poly[static_cast<size_t>(i)].y();
    const double yj = poly[static_cast<size_t>(j)].y();
    const bool intersect = ((yi > y) != (yj > y));
    if (!intersect) {
      continue;
    }
    const double xi = poly[static_cast<size_t>(i)].x();
    const double xj = poly[static_cast<size_t>(j)].x();
    const double den = yj - yi;
    if (x < (xj - xi) * (y - yi) / den + xi) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

GridMap::GridMap()
: latest_snapshot_(std::make_shared<GridMapSnapshot>())
{
  mapping_thread_ = std::thread(&GridMap::mappingLoop, this);
}

GridMap::~GridMap()
{
  {
    std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
    stopping_ = true;
    pending_cloud_.reset();
  }
  pending_cloud_cv_.notify_one();
  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
}

bool GridMap::pointCloudCallback(pcl::PCLPointCloud2::ConstPtr cloud)
{
  if (!cloud) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(pending_cloud_mutex_);
    if (stopping_) {
      return false;
    }
    pending_cloud_ = std::move(cloud);
  }
  pending_cloud_cv_.notify_one();
  return true;
}

std::shared_ptr<const GridMapSnapshot> GridMap::snapshot() const
{
  return std::atomic_load(&latest_snapshot_);
}

void GridMap::mappingLoop()
{
  while (true) {
    pcl::PCLPointCloud2::ConstPtr cloud;
    {
      std::unique_lock<std::mutex> lock(pending_cloud_mutex_);
      pending_cloud_cv_.wait(lock, [this] { return stopping_ || pending_cloud_; });
      if (stopping_) {
        return;
      }
      cloud = std::move(pending_cloud_);
      pending_cloud_.reset();
    }

    processPointCloud(cloud);
    publishSnapshot();
  }
}

void GridMap::processPointCloud(const pcl::PCLPointCloud2::ConstPtr & cloud)
{
  // Implement point-cloud conversion and map updates here.
  (void)cloud;
}

void GridMap::publishSnapshot()
{
  auto next = std::make_shared<GridMapSnapshot>();
  next->geometry = grid_geometry_;
  next->tsdf_voxels = tsdf_voxels_;
  next->occupancy_voxels = occupancy_voxels_;
  next->esdf_voxels = esdf_voxels_;
  next->version = ++snapshot_version_;
  std::atomic_store(
    &latest_snapshot_, std::shared_ptr<const GridMapSnapshot>(std::move(next)));
}
