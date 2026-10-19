#include "plan_env/observation_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace plan_env {
namespace {

using Json = nlohmann::json;

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

class ObservationPlannerNode final : public rclcpp::Node {
public:
  ObservationPlannerNode()
      : Node("plan_env_observation_planner"),
        planner_(readConfig()) {
    task_topic_ = declare_parameter<std::string>(
        "observation.task_topic", "/area_designation");
    cloud_topic_ = declare_parameter<std::string>(
        "grid_map.cloud_topic", "/camera/depth/points");
    target_topic_ = declare_parameter<std::string>(
        "observation.target_topic", "/complex_area_plan/observation_target");
    state_topic_ = declare_parameter<std::string>(
        "observation.state_topic", "/complex_area_plan/observation_state");
    wipe_topic_ = declare_parameter<std::string>(
        "observation.wipe_topic", "/complex_area_plan/wipe_request");
    wipe_complete_topic_ = declare_parameter<std::string>(
        "observation.wipe_complete_topic", "/complex_area_plan/wipe_complete");
    map_frame_ = declare_parameter<std::string>("grid_map.frame_id", "map");
    finish_on_complete_ = declare_parameter<bool>(
        "observation.finish_on_complete", false);
    const int max_stations = declare_parameter<int>(
        "observation.max_stations_per_round", 5);
    const int max_rounds = declare_parameter<int>(
        "observation.max_rounds", 3);
    max_stations_per_observation_ = static_cast<std::size_t>(
        std::max(1, max_stations));
    max_observation_rounds_ = static_cast<std::size_t>(
        std::max(1, max_rounds));
    arrival_tolerance_ = declare_parameter<double>(
        "observation.arrival_tolerance", 0.15);
    tick_period_ms_ = declare_parameter<int>("observation.tick_period_ms", 100);

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        target_topic_, rclcpp::QoS(1).reliable().transient_local());
    state_pub_ = create_publisher<std_msgs::msg::String>(
        state_topic_, rclcpp::QoS(1).reliable().transient_local());
    wipe_pub_ = create_publisher<std_msgs::msg::String>(
        wipe_topic_, rclcpp::QoS(1).reliable().transient_local());
    task_sub_ = create_subscription<std_msgs::msg::String>(
        task_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          handleTask(msg->data);
        });
    wipe_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        wipe_complete_topic_, 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (msg->data) handleWipeComplete();
        });
    tick_timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(20, tick_period_ms_)),
        [this] { tick(); });
  }

  void initialize() {
    map_.initMap(shared_from_this());
    publishState("waiting_for_task");
  }

private:
  enum class Phase { kIdle, kWaitingForMap, kMoving, kWiping, kDone };

  ObservationPlannerConfig readConfig() {
    ObservationPlannerConfig config;
    config.wipe_z_min = declare_parameter<double>(
        "observation.wipe_z_min", config.wipe_z_min);
    config.wipe_z_max = declare_parameter<double>(
        "observation.wipe_z_max", config.wipe_z_max);
    config.arm_reach = declare_parameter<double>(
        "observation.arm_reach", config.arm_reach);
    config.arm_entry_depth = declare_parameter<double>(
        "observation.arm_entry_depth", config.arm_entry_depth);
    config.opening_width = declare_parameter<double>(
        "observation.opening_width", config.opening_width);
    config.sensor_range = declare_parameter<double>(
        "observation.sensor_range", config.sensor_range);
    config.sensor_half_fov = declare_parameter<double>(
        "observation.sensor_half_fov", config.sensor_half_fov);
    config.candidate_spacing = declare_parameter<double>(
        "observation.candidate_spacing", config.candidate_spacing);
    config.candidate_standoff = declare_parameter<double>(
        "observation.candidate_standoff", config.candidate_standoff);
    config.min_wipeable_cells = declare_parameter<double>(
        "observation.min_wipeable_cells", config.min_wipeable_cells);
    config.max_unknown_ratio = declare_parameter<double>(
        "observation.max_unknown_ratio", config.max_unknown_ratio);
    config.min_unknown_gain = declare_parameter<double>(
        "observation.min_unknown_gain", config.min_unknown_gain);
    config.travel_penalty = declare_parameter<double>(
        "observation.travel_penalty", config.travel_penalty);
    const int max_candidates = declare_parameter<int>(
        "observation.max_candidates", static_cast<int>(config.max_candidates));
    config.max_candidates = static_cast<std::size_t>(std::max(1, max_candidates));
    return config;
  }

  static std::vector<Eigen::Vector2d> parsePolygon(const Json &payload,
                                                   std::string *frame) {
    Json points;
    std::string task_frame = frame ? *frame : "map";
    if (payload.value("type", "") == "polygon" &&
        payload.value("function", "") == "clean") {
      const auto &data = payload.at("data");
      task_frame = data.value("frame_id", task_frame);
      points = data.at("points");
    } else if (payload.value("command", "") == "run_complex_area_clean") {
      const auto &polygon = payload.at("polygon");
      task_frame = polygon.value("frame_id", task_frame);
      points = polygon.at("points");
    } else if (payload.value("command", "") == "start") {
      points = payload.at("points");
    } else {
      throw std::runtime_error("unsupported observation task format");
    }
    if (frame) *frame = task_frame;
    if (!points.is_array() || points.size() < 3 || points.size() > 1000) {
      throw std::runtime_error("observation task needs 3..1000 points");
    }
    std::vector<Eigen::Vector2d> polygon_xy;
    polygon_xy.reserve(points.size());
    for (const auto &point : points) {
      const double x = point.at("x").get<double>();
      const double y = point.at("y").get<double>();
      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::runtime_error("polygon contains a non-finite coordinate");
      }
      polygon_xy.emplace_back(x, y);
    }
    return polygon_xy;
  }

  void handleTask(const std::string &raw) {
    try {
      const Json payload = Json::parse(raw);
      if (payload.value("status", 1) == -1 ||
          payload.value("command", "") == "stop") {
        map_.stopTaskMapping();
        active_ = false;
        phase_ = Phase::kIdle;
        publishState("stopped");
        return;
      }
      std::string frame = map_frame_;
      const auto polygon = parsePolygon(payload, &frame);
      if (frame != map_frame_) {
        throw std::runtime_error("task frame_id does not match grid_map.frame_id");
      }
      std::string message;
      if (!map_.startTaskMapping(polygon, &message)) {
        throw std::runtime_error(message.empty() ? "startTaskMapping failed"
                                                 : message);
      }
      planner_.reset();
      polygon_ = polygon;
      visited_targets_.clear();
      last_plan_ = ObservationPlan{};
      target_sent_ = false;
      saw_motion_ = false;
      observation_round_ = 0;
      stations_in_round_ = 0;
      last_counted_position_index_ = std::numeric_limits<uint64_t>::max();
      active_ = true;
      phase_ = Phase::kWaitingForMap;
      publishState("waiting_for_station_commit");
      RCLCPP_INFO(get_logger(), "observation task accepted: %s", message.c_str());
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "observation task rejected: %s", error.what());
      publishState(std::string("rejected:") + error.what());
    }
  }

  void tick() {
    if (!active_ || phase_ == Phase::kWiping || phase_ == Phase::kDone) return;
    const auto snapshot = map_.snapshot();
    if (!snapshot || !snapshot->task_mapping_active) return;
    const auto status = snapshot->mapping_status;
    const std::string status_signature =
        std::to_string(snapshot->version) + ":" +
        std::to_string(status.position_index) + ":" +
        std::to_string(static_cast<int>(status.motion_state)) + ":" +
        std::to_string(status.stationary_frame_count) + ":" +
        (status.ready_to_move ? "1" : "0");
    if (status_signature != last_status_signature_) {
      RCLCPP_INFO(get_logger(),
                  "planner map: version=%lu position=%lu motion_state=%d "
                  "stationary=%u/%u ready=%s esdf_current=%s",
                  snapshot->version, status.position_index,
                  static_cast<int>(status.motion_state),
                  status.stationary_frame_count,
                  status.stationary_frames_required,
                  status.ready_to_move ? "true" : "false",
                  snapshot->esdf_current ? "true" : "false");
      last_status_signature_ = status_signature;
    }
    if (target_sent_) {
      if (status.motion_state == MotionState::kMoving) saw_motion_ = true;
      const Eigen::Vector2d current(snapshot->motion_position.x(),
                                    snapshot->motion_position.y());
      const double distance = (current - last_plan_.target.xy).norm();
      const bool reached = status.ready_to_move &&
                           distance <= arrival_tolerance_ &&
                           (saw_motion_ || !target_requires_motion_);
      if (reached) {
        visited_targets_.push_back(last_plan_.target.xy);
        target_sent_ = false;
        saw_motion_ = false;
        phase_ = Phase::kWaitingForMap;
        publishState("station_observed");
      }
      return;
    }
    if (!status.ready_to_move || status.motion_state != MotionState::kReadyToMove)
      return;

    const Eigen::Vector2d current(snapshot->motion_position.x(),
                                  snapshot->motion_position.y());
    if (status.position_index != last_counted_position_index_) {
      last_counted_position_index_ = status.position_index;
      ++stations_in_round_;
      RCLCPP_INFO(get_logger(), "observation round %zu/%zu station %zu/%zu",
                  observation_round_ + 1, max_observation_rounds_,
                  stations_in_round_, max_stations_per_observation_);
    }
    ObservationPlan next;
    if (!planner_.plan(*snapshot, current, visited_targets_, &next)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "observation plan waiting: %s", next.reason.c_str());
      if (stations_in_round_ < max_stations_per_observation_) return;
      next.valid = true;
      next.complete = next.metrics.wipeable_region_found;
      next.reason = next.complete ? "station_limit_wipeable_region"
                                  : "station_limit_below_wipe_threshold";
    }
    RCLCPP_INFO(get_logger(),
                "observation plan: valid=%s complete=%s reason=%s "
                "unknown_ratio=%.3f opening=%s wipeable=%zu",
                next.valid ? "true" : "false",
                next.complete ? "true" : "false", next.reason.c_str(),
                next.metrics.unknown_ratio,
                next.metrics.opening_found ? "true" : "false",
                next.wipeable_cells.size());
    last_plan_ = next;
    if (next.complete) {
      if (next.reason == "remaining_region_below_minimum") {
        phase_ = Phase::kDone;
        active_ = false;
        publishState("observation_complete");
        completion_timer_ = create_wall_timer(
            std::chrono::milliseconds(1000), [] { rclcpp::shutdown(); });
        return;
      }
      beginWipe(*snapshot);
      return;
    }
    if (stations_in_round_ >= max_stations_per_observation_) {
      phase_ = Phase::kDone;
      active_ = false;
      publishState("observation_aborted_below_wipe_threshold");
      RCLCPP_WARN(get_logger(),
                  "observation stopped after %zu stations: wipeable region "
                  "is below the configured minimum",
                  stations_in_round_);
      completion_timer_ = create_wall_timer(
          std::chrono::milliseconds(1000), [] { rclcpp::shutdown(); });
      return;
    }
    if (next.target.visible_unknown == 0 && next.metrics.unknown_ratio >
                                             planner_.config().max_unknown_ratio) {
      RCLCPP_WARN(get_logger(), "no candidate can reduce unknown area (%s)",
                  next.reason.c_str());
      return;
    }
    publishTarget(next.target);
  }

  void publishTarget(const ObservationTarget &target) {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.pose.position.x = target.xy.x();
    message.pose.position.y = target.xy.y();
    message.pose.orientation = quaternionFromYaw(target.yaw);
    target_pub_->publish(message);
    target_sent_ = true;
    phase_ = Phase::kMoving;
    saw_motion_ = false;
    const auto snapshot = map_.snapshot();
    const Eigen::Vector2d current(snapshot ? snapshot->motion_position.x() : 0.0,
                                  snapshot ? snapshot->motion_position.y() : 0.0);
    target_requires_motion_ = (target.xy - current).norm() > arrival_tolerance_;
    publishState(target_requires_motion_ ? "moving_to_observation"
                                         : "observing_current_station");
  }

  void beginWipe(const EsdfSnapshot &snapshot) {
    map_.freezeOccupancyAndEsdf();
    phase_ = Phase::kWiping;
    Json command;
    command["command"] = "wipe_observed_region";
    command["frame_id"] = map_frame_;
    command["reason"] = last_plan_.reason;
    command["unknown_ratio"] = last_plan_.metrics.unknown_ratio;
    command["opening_found"] = last_plan_.metrics.opening_found;
    command["wipeable_cells"] = last_plan_.wipeable_cells.size();
    command["observation_round"] = observation_round_ + 1;
    command["theoretically_unobservable"] =
        last_plan_.metrics.theoretically_unobservable;
    command["points"] = Json::array();
    for (const auto &point : last_plan_.wipeable_cells) {
      command["points"].push_back({{"x", point.x()}, {"y", point.y()}});
    }
    std_msgs::msg::String message;
    message.data = command.dump();
    wipe_pub_->publish(message);
    publishState("wiping");
  }

  void handleWipeComplete() {
    if (phase_ != Phase::kWiping) return;
    planner_.markWiped(last_plan_);
    map_.clearEsdfFreezing();
    ++observation_round_;
    if (observation_round_ >= max_observation_rounds_) {
      phase_ = Phase::kDone;
      active_ = false;
      publishState("observation_complete");
      completion_timer_ = create_wall_timer(
          std::chrono::milliseconds(1000), [] { rclcpp::shutdown(); });
      return;
    }
    target_sent_ = false;
    saw_motion_ = false;
    visited_targets_.clear();
    stations_in_round_ = 0;
    last_counted_position_index_ = std::numeric_limits<uint64_t>::max();
    phase_ = Phase::kWaitingForMap;
    publishState("assessing_remaining_region");
  }

  void publishState(const std::string &state) {
    std_msgs::msg::String message;
    message.data = state;
    state_pub_->publish(message);
  }

  GridMap map_;
  ObservationPlanner planner_;
  std::vector<Eigen::Vector2d> polygon_;
  std::vector<Eigen::Vector2d> visited_targets_;
  ObservationPlan last_plan_;
  Phase phase_ = Phase::kIdle;
  bool active_ = false;
  bool target_sent_ = false;
  bool target_requires_motion_ = false;
  bool saw_motion_ = false;
  bool finish_on_complete_ = false;
  std::size_t max_stations_per_observation_ = 5;
  std::size_t max_observation_rounds_ = 3;
  std::size_t observation_round_ = 0;
  std::size_t stations_in_round_ = 0;
  uint64_t last_counted_position_index_ =
      std::numeric_limits<uint64_t>::max();
  double arrival_tolerance_ = 0.15;
  int tick_period_ms_ = 100;
  std::string map_frame_ = "map";
  std::string task_topic_;
  std::string cloud_topic_;
  std::string target_topic_;
  std::string state_topic_;
  std::string wipe_topic_;
  std::string wipe_complete_topic_;
  std::string last_status_signature_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wipe_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wipe_complete_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr completion_timer_;
};

}  // namespace
}  // namespace plan_env

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plan_env::ObservationPlannerNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

