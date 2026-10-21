#include "plan_env/grid_map.hpp"
#include "plan_env/height_map.hpp"
#include "standalone_visualization.hpp"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <mutex>
#include <utility>
#include <vector>

#include <sys/stat.h>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace plan_env {
namespace {

#ifndef PLAN_ENV_VIS_DIR
#define PLAN_ENV_VIS_DIR "vis"
#endif

#ifndef PLAN_ENV_BAG_DIR
#define PLAN_ENV_BAG_DIR "plan_env/bag"
#endif

using Json = nlohmann::json;

const char *motionStateName(const MotionState state) {
  switch (state) {
  case MotionState::kWaitingForPose:
    return "waiting_for_pose";
  case MotionState::kMoving:
    return "moving";
  case MotionState::kStationaryCollecting:
    return "stationary_collecting";
  case MotionState::kReadyToMove:
    return "ready_to_move";
  }
  return "unknown";
}

bool ensureDirectory(const std::string &path) {
  if (path.empty()) {
    return false;
  }
  std::string current;
  size_t offset = 0;
  if (path.front() == '/') {
    current = "/";
    offset = 1;
  }
  while (offset <= path.size()) {
    const size_t separator = path.find('/', offset);
    const std::string part = path.substr(offset, separator == std::string::npos
                                                     ? std::string::npos
                                                     : separator - offset);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/') {
        current += '/';
      }
      current += part;
      if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }
    if (separator == std::string::npos) {
      break;
    }
    offset = separator + 1;
  }
  return true;
}

bool writePpm(const std::string &path, const uint32_t width,
              const uint32_t height, const std::vector<uint8_t> &rgb) {
  if (width == 0 || height == 0 ||
      rgb.size() != static_cast<size_t>(width) * height * 3) {
    return false;
  }
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    return false;
  }
  output << "P6\n" << width << ' ' << height << "\n255\n";
  output.write(reinterpret_cast<const char *>(rgb.data()),
               static_cast<std::streamsize>(rgb.size()));
  return output.good();
}

void setPixel(std::vector<uint8_t> &rgb, const size_t index, const uint8_t red,
              const uint8_t green, const uint8_t blue) {
  const size_t offset = index * 3;
  rgb[offset] = red;
  rgb[offset + 1] = green;
  rgb[offset + 2] = blue;
}

sensor_msgs::msg::PointCloud2 toPointCloud2(
    const pcl::PointCloud<pcl::PointXYZI> &cloud,
    const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 message;
  if (cloud.empty()) {
    message.height = 1;
    message.width = 0;
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = sizeof(float) * 4;
    message.fields.resize(4);
    const char *names[] = {"x", "y", "z", "intensity"};
    for (std::size_t i = 0; i < 4; ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
      message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[i].count = 1;
    }
  } else {
    pcl::toROSMsg(cloud, message);
  }
  message.header = header;
  return message;
}

class GridMapStandaloneTest : public rclcpp::Node {
public:
  GridMapStandaloneTest() : Node("grid_map_standalone_test") {
    map_ = std::make_unique<GridMap>();

    const std::string task_topic =
        declare_parameter<std::string>("task_topic", "/area_designation");
    const std::string height_topic = declare_parameter<std::string>(
        "heightmap_topic", "/grid_map/heightmap");
    const std::string esdf_topic =
        declare_parameter<std::string>("esdf_topic", "/grid_map/esdf_slice");
    const std::string occupancy_3d_topic = declare_parameter<std::string>(
        "occupancy_3d_topic", "/grid_map/occupied_voxels");
    esdf_slice_z_ = declare_parameter<double>("esdf_slice_z", 0.10);
    visualization_dir_ =
        declare_parameter<std::string>("visualization_dir", PLAN_ENV_VIS_DIR);
    map_frame_ = declare_parameter<std::string>("grid_map.frame_id", "map");
    cloud_topic_ = declare_parameter<std::string>("grid_map.cloud_topic",
                                                  "/camera/depth/points");
    record_bag_ = declare_parameter<bool>("record_bag", true);
    bag_directory_ =
        declare_parameter<std::string>("bag_directory", PLAN_ENV_BAG_DIR);
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
    tf_static_topic_ =
        declare_parameter<std::string>("tf_static_topic", "/tf_static");

    initializeBagWriter();

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    height_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(height_topic, latched);
    esdf_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(esdf_topic, latched);
    occupancy_3d_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        occupancy_3d_topic, latched);
    mapping_status_pub_ = create_publisher<std_msgs::msg::String>(
        "/grid_map/mapping_status", latched);
    task_sub_ = create_subscription<std_msgs::msg::String>(
        task_topic, latched,
        [this](const std_msgs::msg::String::ConstSharedPtr message) {
          onTask(message->data);
        });
    auto cloud_qos = rclcpp::QoS(10);
    if (declare_parameter<bool>("grid_map.cloud_reliable", true)) {
      cloud_qos.reliable();
    } else {
      cloud_qos.best_effort();
    }
    cloud_monitor_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          ++cloud_messages_total_;
          ++cloud_messages_for_task_;
          last_cloud_frame_ = message->header.frame_id;
          last_cloud_point_count_ =
              static_cast<uint64_t>(message->width) * message->height;
          // Forward the received sample through GridMap's thread-safe mailbox
          // as a fallback for large fragmented DDS samples that can bypass
          // the library subscription while the monitor still receives them.
          if (map_) {
            map_->pointCloudCallback(message);
          }
          recordBagMessage(*message, cloud_topic_, message->header.stamp);
        });
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, rclcpp::QoS(100).best_effort(),
        [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          recordBagMessage(*message, tf_topic_, tfMessageStamp(*message));
        });
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_static_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          recordBagMessage(*message, tf_static_topic_,
                           tfMessageStamp(*message));
        });
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                       [this] { publishMaps(); });

    RCLCPP_INFO(get_logger(),
                "standalone GridMap ready: task=%s cloud=%s "
                "heightmap=%s esdf=%s occupancy3d=%s",
                task_topic.c_str(), cloud_topic_.c_str(), height_topic.c_str(),
                esdf_topic.c_str(), occupancy_3d_topic.c_str());
  }

  void initializeMap() {
    map_->initMap(shared_from_this());
    initialized_ = true;
  }

private:
  void onTask(const std::string &raw) {
    if (!initialized_) {
      RCLCPP_WARN(get_logger(), "task received before GridMap initialization");
      return;
    }
    try {
      const Json payload = Json::parse(raw);
      if (!payload.is_object()) {
        throw std::runtime_error("task JSON root must be an object");
      }
      if (payload.value("status", 1) == -1 ||
          payload.value("command", "") == "stop") {
        map_->stopTaskMapping();
        task_active_ = false;
        RCLCPP_INFO(get_logger(), "task mapping stopped");
        return;
      }

      Json points;
      std::string task_frame = payload.value("frame_id", map_frame_);
      if (payload.value("type", "") == "polygon" &&
          payload.value("function", "") == "clean") {
        const auto &data = payload.at("data");
        task_frame = data.value("frame_id", task_frame);
        points = data.at("points");
      } else if (payload.value("command", "") == "run_complex_area_clean") {
        const auto &polygon_data = payload.at("polygon");
        task_frame = polygon_data.value("frame_id", task_frame);
        points = polygon_data.at("points");
      } else if (payload.value("command", "") == "start") {
        points = payload.at("points");
      } else {
        throw std::runtime_error("unsupported task format");
      }
      if (task_frame != map_frame_) {
        throw std::runtime_error("task frame_id must match grid_map.frame_id");
      }
      if (!points.is_array() || points.size() < 3 || points.size() > 1000) {
        throw std::runtime_error("task needs 3..1000 polygon points");
      }
      std::vector<Eigen::Vector2d> polygon;
      polygon.reserve(points.size());
      for (const auto &point : points) {
        const double x = point.at("x").get<double>();
        const double y = point.at("y").get<double>();
        if (!std::isfinite(x) || !std::isfinite(y)) {
          throw std::runtime_error("polygon contains a non-finite coordinate");
        }
        polygon.emplace_back(x, y);
      }
      std::string message;
      if (!map_->startTaskMapping(polygon, &message)) {
        throw std::runtime_error(message.empty() ? "startTaskMapping failed"
                                                 : message);
      }
      task_active_ = true;
      cloud_messages_for_task_ = 0;
      last_wait_log_ = std::chrono::steady_clock::time_point{};
      const auto status = map_->mappingStatus();
      RCLCPP_INFO(get_logger(),
                  "%s; waiting for cloud fusion (cloud=%s, "
                  "stationary_frames=%u)",
                  message.c_str(), cloud_topic_.c_str(),
                  status.stationary_frames_required);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "task rejected: %s", error.what());
    }
  }

  void publishMaps() {
    if (!initialized_) {
      return;
    }
    const auto mapping_status = map_->mappingStatus();
    Json status_json;
    status_json["ready_to_move"] = mapping_status.ready_to_move;
    status_json["position_index"] = mapping_status.position_index;
    status_json["stationary_frame_count"] =
        mapping_status.stationary_frame_count;
    status_json["stationary_frames_required"] =
        mapping_status.stationary_frames_required;
    status_json["motion_state"] = motionStateName(mapping_status.motion_state);
    std_msgs::msg::String status_message;
    status_message.data = status_json.dump();
    mapping_status_pub_->publish(status_message);
    const auto snapshot = map_->snapshot();
    if (!snapshot || !snapshot->task_mapping_active) {
      logWaitingForSnapshot(snapshot);
      return;
    }
    const auto &geometry = snapshot->geometry;
    if (geometry.size_x <= 0 || geometry.size_y <= 0 || geometry.size_z <= 0) {
      return;
    }

    nav_msgs::msg::OccupancyGrid height;
    height.header.stamp = now();
    height.header.frame_id = map_frame_;
    height.info.map_load_time = height.header.stamp;
    height.info.resolution = geometry.resolution;
    height.info.width = static_cast<uint32_t>(geometry.size_x);
    height.info.height = static_cast<uint32_t>(geometry.size_y);
    height.info.origin.position.x = geometry.origin.x();
    height.info.origin.position.y = geometry.origin.y();
    height.info.origin.position.z = 0.0;
    height.info.origin.orientation.w = 1.0;
    height.data.assign(static_cast<size_t>(geometry.size_x) * geometry.size_y,
                       -1);

    pcl::PointCloud<pcl::PointXYZI> esdf;
    pcl::PointCloud<pcl::PointXYZI> occupied_voxels;
    const int slice =
        std::max(0, std::min(geometry.size_z - 1,
                             static_cast<int>(std::floor(
                                 (esdf_slice_z_ - geometry.origin.z()) /
                                 geometry.resolution))));
    const int height_z_max = std::max(
        0, std::min(geometry.size_z - 1,
                    static_cast<int>(std::floor(0.50 / geometry.resolution))));
    const HeightBandParams height_band{};
    for (int y = 0; y < geometry.size_y; ++y) {
      for (int x = 0; x < geometry.size_x; ++x) {
        bool has_free = false;
        bool has_occupied = false;
        double lowest_occupied = std::numeric_limits<double>::infinity();
        for (int z = 0; z <= height_z_max; ++z) {
          const size_t address = static_cast<size_t>(
              (x * geometry.size_y + y) * geometry.size_z + z);
          if (address >= snapshot->occupancy_states.size()) {
            continue;
          }
          const auto state = snapshot->occupancy_states[address];
          if (state == OccupancyState::FREE) {
            has_free = true;
          } else if (state == OccupancyState::OCCUPIED) {
            const double relative_z = (z + 0.5) * geometry.resolution;
            if (occupiedCountsForHeight(relative_z, height_band)) {
              has_occupied = true;
              lowest_occupied = std::min(lowest_occupied, relative_z);
            }
          }
        }
        const size_t grid_index = static_cast<size_t>(y) * geometry.size_x + x;
        const uint8_t status = classifyHeightColumn(
            has_occupied, has_free, lowest_occupied, height_band);
        switch (status) {
        case HeightStatus::kAllOpen:
          height.data[grid_index] = 0;
          break;
        case HeightStatus::kBaseOpen:
          height.data[grid_index] = 25;
          break;
        case HeightStatus::kToolOpen:
          height.data[grid_index] = 50;
          break;
        case HeightStatus::kTooLow:
          height.data[grid_index] = 75;
          break;
        case HeightStatus::kNotFloor:
          height.data[grid_index] = 100;
          break;
        default:
          height.data[grid_index] = -1;
          break;
        }

        const size_t address = static_cast<size_t>(
            (x * geometry.size_y + y) * geometry.size_z + slice);
        if (address >= snapshot->esdf_voxels.size()) {
          continue;
        }
        pcl::PointXYZI point;
        point.x = geometry.origin.x() + (x + 0.5f) * geometry.resolution;
        point.y = geometry.origin.y() + (y + 0.5f) * geometry.resolution;
        point.z = geometry.origin.z() + (slice + 0.5f) * geometry.resolution;
        point.intensity = snapshot->esdf_voxels[address].distance;
        esdf.push_back(point);

        for (int z = 0; z < geometry.size_z; ++z) {
          const size_t voxel_address = static_cast<size_t>(
              (x * geometry.size_y + y) * geometry.size_z + z);
          if (voxel_address >= snapshot->occupancy_states.size() ||
              snapshot->occupancy_states[voxel_address] !=
                  OccupancyState::OCCUPIED) {
            continue;
          }
          pcl::PointXYZI occupied;
          occupied.x = geometry.origin.x() + (x + 0.5f) * geometry.resolution;
          occupied.y = geometry.origin.y() + (y + 0.5f) * geometry.resolution;
          occupied.z = geometry.origin.z() + (z + 0.5f) * geometry.resolution;
          occupied.intensity = snapshot->occupancy_probabilities[voxel_address];
          occupied_voxels.push_back(occupied);
        }
      }
    }

    height_pub_->publish(height);
    esdf.width = static_cast<uint32_t>(esdf.size());
    esdf.height = 1;
    esdf.is_dense = true;
    sensor_msgs::msg::PointCloud2 esdf_message =
        toPointCloud2(esdf, height.header);
    esdf_pub_->publish(esdf_message);

    sensor_msgs::msg::PointCloud2 occupancy_message =
        toPointCloud2(occupied_voxels, height.header);
    occupancy_3d_pub_->publish(occupancy_message);

    if (snapshot->version != last_visualized_version_ &&
        saveVisualizations(*snapshot, height, geometry)) {
      last_visualized_version_ = snapshot->version;
    }

    uint64_t unknown_cells = 0;
    uint64_t open_cells = 0;
    uint64_t restricted_cells = 0;
    for (const int8_t cell : height.data) {
      if (cell < 0) {
        ++unknown_cells;
      } else if (cell == 0) {
        ++open_cells;
      } else {
        ++restricted_cells;
      }
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Foxglove maps published: version=%lu grid=%ux%u@%.3fm "
        "height_cells(unknown/open/restricted)=%lu/%lu/%lu "
        "esdf_points=%u slice_z=%.3fm subscriptions(height/esdf)=%zu/%zu "
        "cloud_messages(task/total)=%lu/%lu",
        static_cast<unsigned long>(snapshot->version), height.info.width,
        height.info.height, height.info.resolution,
        static_cast<unsigned long>(unknown_cells),
        static_cast<unsigned long>(open_cells),
        static_cast<unsigned long>(restricted_cells), esdf.width, esdf_slice_z_,
        height_pub_->get_subscription_count(),
        esdf_pub_->get_subscription_count(),
        static_cast<unsigned long>(cloud_messages_for_task_),
        static_cast<unsigned long>(cloud_messages_total_));
  }

  template <typename MessageT>
  void recordBagMessage(const MessageT &message, const std::string &topic,
                        const builtin_interfaces::msg::Time &stamp) {
    if (!bag_writer_) {
      return;
    }
    std::lock_guard<std::mutex> lock(bag_mutex_);
    try {
      const rclcpp::Time message_time(stamp);
      bag_writer_->write(message, topic,
                         message_time.nanoseconds() > 0 ? message_time : now());
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "failed to write bag message on %s: %s",
                            topic.c_str(), error.what());
    }
  }

  static builtin_interfaces::msg::Time tfMessageStamp(
      const tf2_msgs::msg::TFMessage &message) {
    builtin_interfaces::msg::Time stamp;
    for (const auto &transform : message.transforms) {
      if (transform.header.stamp.sec > stamp.sec ||
          (transform.header.stamp.sec == stamp.sec &&
           transform.header.stamp.nanosec > stamp.nanosec)) {
        stamp = transform.header.stamp;
      }
    }
    return stamp;
  }

  void initializeBagWriter() {
    if (!record_bag_) {
      return;
    }
    if (!ensureDirectory(bag_directory_)) {
      RCLCPP_ERROR(get_logger(), "cannot create bag directory: %s",
                   bag_directory_.c_str());
      return;
    }
    const auto timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    bag_uri_ = bag_directory_ + "/grid_map_" + std::to_string(timestamp);
    try {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_uri_;
      storage_options.storage_id = "sqlite3";
      storage_options.max_cache_size = 0;
      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
      bag_writer_->open(storage_options, converter_options);
      RCLCPP_INFO(get_logger(), "recording point cloud and TF to rosbag: %s",
                  bag_uri_.c_str());
    } catch (const std::exception &error) {
      bag_writer_.reset();
      RCLCPP_ERROR(get_logger(), "cannot open rosbag at %s: %s",
                   bag_uri_.c_str(), error.what());
    }
  }

  bool saveVisualizations(const EsdfSnapshot &snapshot,
                          const nav_msgs::msg::OccupancyGrid &height,
                          const GridGeometry &geometry) {
    if (!ensureDirectory(visualization_dir_)) {
      RCLCPP_ERROR(get_logger(), "cannot create visualization directory: %s",
                   visualization_dir_.c_str());
      return false;
    }

    const uint32_t width = height.info.width;
    const uint32_t image_height = height.info.height;
    std::vector<uint8_t> height_rgb(static_cast<size_t>(width) * image_height *
                                    3);
    for (uint32_t y = 0; y < image_height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const size_t source = static_cast<size_t>(y) * width + x;
        const size_t target =
            static_cast<size_t>(image_height - 1 - y) * width + x;
        switch (height.data[source]) {
        case 0:
          setPixel(height_rgb, target, 47, 158, 68);
          break;
        case 25:
          setPixel(height_rgb, target, 116, 192, 252);
          break;
        case 50:
          setPixel(height_rgb, target, 255, 212, 59);
          break;
        case 75:
          setPixel(height_rgb, target, 240, 62, 62);
          break;
        case 100:
          setPixel(height_rgb, target, 166, 30, 77);
          break;
        default:
          setPixel(height_rgb, target, 151, 117, 250);
          break;
        }
      }
    }

    std::ostringstream stem;
    stem << "map_v" << snapshot.version;
    const std::string height_path =
        visualization_dir_ + "/" + stem.str() + "_heightmap.ppm";
    const std::string esdf_data_path =
        visualization_dir_ + "/" + stem.str() + "_esdf3d.json";
    const std::string esdf_html_path =
        visualization_dir_ + "/" + stem.str() + "_esdf3d.html";
    if (!writePpm(height_path, width, image_height, height_rgb)) {
      RCLCPP_ERROR(get_logger(),
                   "failed to write heightmap image for version %lu",
                   static_cast<unsigned long>(snapshot.version));
      return false;
    }

    const size_t voxel_count = static_cast<size_t>(geometry.size_x) *
                               geometry.size_y * geometry.size_z;
    Json esdf_data;
    esdf_data["version"] = snapshot.version;
    esdf_data["frame_id"] = map_frame_;
    esdf_data["origin"] = {geometry.origin.x(), geometry.origin.y(),
                           geometry.origin.z()};
    esdf_data["resolution"] = geometry.resolution;
    esdf_data["dimensions"] = {geometry.size_x, geometry.size_y,
                               geometry.size_z};
    esdf_data["layout"] = "x-major, then y, then z";
    esdf_data["esdf_current"] = snapshot.esdf_current;
    esdf_data["heightmap"] = height.data;
    esdf_data["heightmap_layout"] = "row-major: y * width + x";
    esdf_data["roi"] = Json::array();
    const auto &roi = snapshot.source_roi_polygon.size() >= 3
                          ? snapshot.source_roi_polygon
                          : snapshot.original_roi_polygon;
    for (const auto &point : roi) {
      esdf_data["roi"].push_back({point.x(), point.y()});
    }
    esdf_data["occupancy_semantics"] =
        "1=occupied (distance<=0), 0=free (distance>0), null=unknown";
    esdf_data["occupancy_state_semantics"] =
        "ASCII 0=free, 1=occupied, 2=unknown";
    Json occupancy = Json::array();
    occupancy.get_ref<Json::array_t &>().reserve(voxel_count);
    for (size_t index = 0; index < voxel_count; ++index) {
      if (index < snapshot.esdf_voxels.size() &&
          std::isfinite(snapshot.esdf_voxels[index].distance)) {
        // ESDF zero and negative distances are occupied; positive distances
        // are known free space. Keep invalid values as unknown.
        occupancy.push_back(snapshot.esdf_voxels[index].distance <= 0.0f ? 1
                                                                         : 0);
      } else {
        occupancy.push_back(nullptr);
      }
    }
    esdf_data["occupancy"] = std::move(occupancy);
    std::string occupancy_states;
    occupancy_states.reserve(voxel_count);
    for (size_t index = 0; index < voxel_count; ++index) {
      if (index < snapshot.occupancy_states.size()) {
        occupancy_states.push_back(static_cast<char>(
            '0' + static_cast<int>(snapshot.occupancy_states[index])));
      } else {
        occupancy_states.push_back('2');
      }
    }
    esdf_data["occupancy_state"] = std::move(occupancy_states);
    const std::string esdf_distance_path =
        visualization_dir_ + "/" + stem.str() + "_esdf_distance.f32";
    std::ofstream distance_output(esdf_distance_path, std::ios::binary);
    if (!distance_output) {
      RCLCPP_ERROR(get_logger(), "cannot write ESDF distances: %s",
                   esdf_distance_path.c_str());
      return false;
    }
    for (size_t index = 0; index < voxel_count; ++index) {
      const float distance =
          index < snapshot.esdf_voxels.size() &&
                  std::isfinite(snapshot.esdf_voxels[index].distance)
              ? snapshot.esdf_voxels[index].distance
              : std::numeric_limits<float>::quiet_NaN();
      static_assert(sizeof(float) == sizeof(uint32_t),
                    "ESDF binary output requires 32-bit floats");
      uint32_t bits;
      std::memcpy(&bits, &distance, sizeof(bits));
      const uint8_t bytes[] = {static_cast<uint8_t>(bits & 0xff),
                               static_cast<uint8_t>((bits >> 8) & 0xff),
                               static_cast<uint8_t>((bits >> 16) & 0xff),
                               static_cast<uint8_t>((bits >> 24) & 0xff)};
      distance_output.write(reinterpret_cast<const char *>(bytes), 4);
    }
    if (!distance_output.good()) {
      return false;
    }
    esdf_data["esdf_distance_file"] = stem.str() + "_esdf_distance.f32";
    const std::string data_json = esdf_data.dump();
    {
      std::ofstream output(esdf_data_path);
      if (!output) {
        RCLCPP_ERROR(get_logger(), "cannot write ESDF data: %s",
                     esdf_data_path.c_str());
        return false;
      }
      output << std::setprecision(9) << data_json;
      if (!output.good()) {
        return false;
      }
    }

    {
      std::ofstream output(esdf_html_path);
      if (!output) {
        RCLCPP_ERROR(get_logger(), "cannot write ESDF HTML: %s",
                     esdf_html_path.c_str());
        return false;
      }
      writeStandaloneVisualization(output, data_json);
      if (!output.good()) {
        return false;
      }
    }

    const std::string manifest_path = visualization_dir_ + "/versions_3d.csv";
    std::ofstream manifest(manifest_path, std::ios::app);
    if (!manifest) {
      RCLCPP_ERROR(get_logger(), "cannot append visualization index: %s",
                   manifest_path.c_str());
      return false;
    }
    if (manifest.tellp() == 0) {
      manifest
          << "version,heightmap,esdf_data,esdf_html,width,height,resolution,"
             "esdf_voxels\n";
    }
    manifest << snapshot.version << ',' << stem.str() << "_heightmap.ppm,"
             << stem.str() << "_esdf3d.json," << stem.str() << "_esdf3d.html,"
             << width << ',' << image_height << ',' << std::setprecision(9)
             << geometry.resolution << ',' << voxel_count << '\n';
    RCLCPP_INFO(get_logger(), "saved map visualization version=%lu to %s",
                static_cast<unsigned long>(snapshot.version),
                visualization_dir_.c_str());
    return true;
  }

  void logWaitingForSnapshot(
      const std::shared_ptr<const EsdfSnapshot> &snapshot) {
    if (!task_active_) {
      return;
    }
    const auto current_time = std::chrono::steady_clock::now();
    if (last_wait_log_ != std::chrono::steady_clock::time_point{} &&
        current_time - last_wait_log_ < std::chrono::seconds(5)) {
      return;
    }
    last_wait_log_ = current_time;
    const auto status = map_->mappingStatus();
    const uint64_t snapshot_version = snapshot ? snapshot->version : 0;
    if (cloud_messages_for_task_ == 0) {
      RCLCPP_WARN(get_logger(),
                  "no PointCloud2 received for active task: topic=%s "
                  "total_cloud_messages=%lu; map topics are waiting for "
                  "cloud fusion (stationary_frames_required=%u)",
                  cloud_topic_.c_str(),
                  static_cast<unsigned long>(cloud_messages_total_),
                  status.stationary_frames_required);
      return;
    }
    RCLCPP_WARN(get_logger(),
                "task has no map snapshot yet: cloud_messages(task/total)="
                "%lu/%lu last_cloud=%s[%lu points] "
                "stationary_frames=%u/%u motion_state=%s snapshot_version=%lu "
                "task_mapping_active=%s",
                static_cast<unsigned long>(cloud_messages_for_task_),
                static_cast<unsigned long>(cloud_messages_total_),
                last_cloud_frame_.c_str(),
                static_cast<unsigned long>(last_cloud_point_count_),
                status.stationary_frame_count,
                status.stationary_frames_required,
                motionStateName(status.motion_state),
                static_cast<unsigned long>(snapshot_version),
                snapshot && snapshot->task_mapping_active ? "true" : "false");
  }

  std::unique_ptr<GridMap> map_;
  bool initialized_{false};
  double esdf_slice_z_{0.10};
  std::string visualization_dir_;
  uint64_t last_visualized_version_{0};
  std::string map_frame_{"map"};
  std::string cloud_topic_{"/camera/depth/points"};
  std::string tf_topic_{"/tf"};
  std::string tf_static_topic_{"/tf_static"};
  std::string bag_directory_{"plan_env/bag"};
  std::string bag_uri_;
  bool record_bag_{true};
  std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;
  std::mutex bag_mutex_;
  std::string last_cloud_frame_;
  uint64_t cloud_messages_total_{0};
  uint64_t cloud_messages_for_task_{0};
  uint64_t last_cloud_point_count_{0};
  bool task_active_{false};
  std::chrono::steady_clock::time_point last_wait_log_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_monitor_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr height_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_3d_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapping_status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace
}  // namespace plan_env

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plan_env::GridMapStandaloneTest>();
  node->initializeMap();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
