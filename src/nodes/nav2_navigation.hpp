#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace plan_env {

// Driven by the node's single-threaded executor. Like cleaning/orchestrator.py,
// each operation has a generation so late responses cannot revive a stopped
// task.
class Nav2Navigation {
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Clock = std::chrono::steady_clock;
  enum class State { Idle, WaitingForServer, Navigating, Succeeded, Failed };

  Nav2Navigation(rclcpp::Node *node, const std::string &action,
                 double server_timeout, double navigation_timeout)
      : node_(node), server_timeout_(server_timeout),
        navigation_timeout_(navigation_timeout) {
    if (!std::isfinite(server_timeout) || server_timeout <= 0.0 ||
        !std::isfinite(navigation_timeout) || navigation_timeout <= 0.0) {
      throw std::invalid_argument(
          "navigation timeouts must be finite and positive");
    }
    client_ = rclcpp_action::create_client<Action>(node, action);
  }

  void start(const geometry_msgs::msg::PoseStamped &pose) {
    cancel();
    goal_.pose = pose;
    goal_.behavior_tree =
        "";  // Use the Nav2 server's configured behavior tree.
    state_ = State::WaitingForServer;
    deadline_ = Clock::now() + duration(server_timeout_);
    tick();
  }

  void cancel() {
    ++generation_;
    if (handle_) {
      cancelHandle(handle_);
      handle_.reset();
    }
    state_ = State::Idle;
    error_.clear();
  }

  void tick() {
    if (state_ != State::WaitingForServer && state_ != State::Navigating) {
      return;
    }
    if (Clock::now() >= deadline_) {
      fail(state_ == State::WaitingForServer ? "server_timeout" : "timeout");
      return;
    }
    if (state_ != State::WaitingForServer ||
        !client_->action_server_is_ready()) {
      return;
    }
    state_ = State::Navigating;
    deadline_ = Clock::now() + duration(navigation_timeout_);
    goal_.pose.header.stamp = node_->now();
    const auto generation = generation_;
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback =
        [this, generation](GoalHandle::SharedPtr handle) {
          if (generation != generation_) {
            if (handle) {
              cancelHandle(handle);
            }
            return;
          }
          if (!handle) {
            fail("rejected");
            return;
          }
          handle_ = handle;
        };
    options.result_callback = [this, generation](
                                  const GoalHandle::WrappedResult &result) {
      if (generation != generation_) {
        return;
      }
      handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        state_ = State::Succeeded;
      } else {
        fail(result.code == rclcpp_action::ResultCode::CANCELED ? "canceled"
                                                                : "aborted");
      }
    };
    try {
      client_->async_send_goal(goal_, options);
    } catch (const std::exception &error) {
      fail(std::string("send_failed:") + error.what());
    }
  }

  State state() const { return state_; }
  const std::string &error() const { return error_; }

private:
  static Clock::duration duration(double seconds) {
    return std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(seconds));
  }

  void cancelHandle(const GoalHandle::SharedPtr &handle) {
    try {
      client_->async_cancel_goal(handle);
    } catch (const std::exception &error) {
      RCLCPP_WARN(node_->get_logger(), "Nav2 cancel failed: %s", error.what());
    }
  }

  void fail(const std::string &reason) {
    cancel();
    state_ = State::Failed;
    error_ = reason;
  }

  rclcpp::Node *node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  GoalHandle::SharedPtr handle_;
  Action::Goal goal_;
  State state_ = State::Idle;
  uint64_t generation_ = 0;
  double server_timeout_;
  double navigation_timeout_;
  Clock::time_point deadline_;
  std::string error_;
};

}  // namespace plan_env
