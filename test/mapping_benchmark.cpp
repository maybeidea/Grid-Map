// Deterministic in-process benchmark: real PCL/TF/fusion/snapshot paths, no DDS
// input scheduling. CPU time includes all worker threads; 100% is one core.
#include "plan_env/grid_map.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>

namespace plan_env {
struct GridMapTestAccess {
  static void frame(GridMap &map,
                    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud,
                    bool moving, int frame) {
    std::lock_guard<std::recursive_mutex> lock(map.task_region_mutex_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_footprint";
    tf.header.stamp = rclcpp::Time(frame + 1, 0, RCL_ROS_TIME);
    tf.transform.rotation.w = 1;
    tf.transform.translation.x = moving ? (frame % 2) * 0.1 : 0;
    map.tf_buffer_->setTransform(tf, "benchmark", false);
    cloud->header.stamp = tf.header.stamp;
    map.processQueuedCloud(cloud, map.task_generation_);
  }
};
}  // namespace plan_env

namespace {
using Clock = std::chrono::steady_clock;
template <typename F> void measure(const std::string &name, int count, F run) {
  std::vector<double> samples;
  const auto cpu_start = std::clock();
  const auto begin = Clock::now();
  for (int i = 0; i < count; ++i) {
    const auto start = Clock::now();
    run(i);
    samples.push_back(
        std::chrono::duration<double, std::milli>(Clock::now() - start)
            .count());
  }
  const double wall =
      std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
  const double cpu = 1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC;
  std::sort(samples.begin(), samples.end());
  std::cout << name << ',' << count << ',' << wall / count << ','
            << samples[count / 2] << ','
            << samples[std::min(count - 1, int(count * .95))] << ','
            << cpu / count << ',' << 100 * cpu / wall << '\n';
}

void esdfBench(int count) {
  for (const auto &dims : std::vector<Eigen::Vector3i>{
           {50, 50, 50}, {70, 70, 17}, {200, 200, 34}}) {
    plan_env::GridGeometry geometry;
    geometry.size_x = dims.x();
    geometry.size_y = dims.y();
    geometry.size_z = dims.z();
    geometry.resolution = .015f;
    plan_env::occupancy::LogOddsModel model;
    std::vector<plan_env::OccupancyVoxel> occupancy(dims.prod());
    std::mt19937 random(42);
    for (auto &voxel : occupancy) {
      model.update(voxel, random() % 20 == 0);
    }
    plan_env::esdf::EsdfConfig config;
    config.unknown_is_occupied = false;
    plan_env::esdf::EsdfVolume volume;
    if (!volume.configure(geometry, config)) {
      throw std::runtime_error("ESDF configuration failed");
    }
    for (int i = 0; i < 3; ++i) {
      volume.compute(occupancy, model);
    }
    const std::string suffix = std::to_string(dims.prod());
    measure("esdf_forced_" + suffix, count, [&](int i) {
      occupancy[0].log_odds = i % 2 ? 1.f : -1.f;
      volume.compute(occupancy, model);
    });
    measure("esdf_unchanged_" + suffix, count, [&](int) {
#ifdef PLAN_ENV_BENCHMARK_SOURCE_CACHE
      volume.computeIfChanged(occupancy, model);
#else
      volume.compute(occupancy, model);
#endif
    });
#ifdef PLAN_ENV_BENCHMARK_SOURCE_CACHE
    for (bool local : {false, true}) {
      config.local_update = local;
      volume.configure(geometry, config);
      occupancy[0].log_odds = -1;
      volume.computeIfChanged(occupancy, model);
      measure(std::string(local ? "esdf_local_sparse_" : "esdf_full_sparse_") + suffix,
              count, [&](int i) {
        occupancy[0].log_odds = i % 2 ? -1.f : 1.f;
        volume.computeIfChanged(occupancy, model);
      });
      std::cerr << "local=" << local << " voxels=" << occupancy.size()
                << " updated_lines=" << volume.lastUpdatedLines() << '\n';
    }
#endif

  }
}

void pipelineBench(int count) {
  auto makeCloud = [](bool changed) {
    pcl::PointCloud<pcl::PointXYZ> points;
    for (int x = 0; x < 100; ++x) {
      for (int y = 0; y < 100; ++y) {
        const float px = -0.99f + x * .02f, py = -0.99f + y * .02f;
        const float z =
            changed && std::abs(px) < .3f && std::abs(py) < .3f ? .30f : .12f;
        points.emplace_back(px, py, z);
      }
    }
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(points, *msg);
    msg->header.frame_id = "map";
    return msg;
  };
  auto normal = makeCloud(false), changed = makeCloud(true);
  for (int moving = 0; moving < 2; ++moving) {
    plan_env::GridMap map;
    if (!map.configureDualResolution(.01f, .03f, .5f, .8f)) {
      throw std::runtime_error("map configuration failed");
    }
    map.configureMotion(.03f, .02f, 30);
    plan_env::registration::Config registration;
    registration.enabled = false;
    map.configureRegistration(registration);
    if (!map.startTaskMapping(
            {{-.25, -.25}, {.25, -.25}, {.25, .25}, {-.25, .25}})) {
      throw std::runtime_error("map start failed");
    }
    for (int i = 0; i < 35; ++i) {
      plan_env::GridMapTestAccess::frame(map, normal, false, i);
    }
    measure(moving ? "pipeline_changing_moving" : "pipeline_steady", count,
            [&](int i) {
              plan_env::GridMapTestAccess::frame(
                  map, moving && i % 2 ? changed : normal, moving, i + 35);
            });
    const auto snapshot = map.snapshot();
    double checksum = 0;
    for (const auto &v : snapshot->expanded_occupancy_voxels) {
      checksum += v.log_odds;
    }
    for (const auto &v : snapshot->expanded_esdf_voxels) {
      checksum += v.distance;
    }
    std::cerr << "checksum=" << std::setprecision(12) << checksum
              << " high=" << snapshot->occupancy_voxels.size()
              << " low=" << snapshot->expanded_occupancy_voxels.size() << '\n';
  }
}
}  // namespace

int main(int argc, char **argv) {
  const int count = argc > 2 ? std::max(1, std::stoi(argv[2])) : 40;
  const std::string mode = argc > 1 ? argv[1] : "all";
  std::cout << std::fixed << std::setprecision(4)
            << "case,iterations,wall_mean_ms,wall_median_ms,wall_p95_ms,cpu_ms,"
               "cpu_percent\n";
  if (mode != "pipeline") {
    esdfBench(count);
  }
  if (mode != "esdf") {
    rclcpp::init(0, nullptr);
    pipelineBench(count);
    rclcpp::shutdown();
  }
}
