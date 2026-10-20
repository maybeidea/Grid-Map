#include "nav2_navigation.hpp"

#include <functional>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace plan_env {
namespace {
class Nav2NavigationTest : public ::testing::Test {
protected:
  using Action = Nav2Navigation::Action;
  using ServerHandle = rclcpp_action::ServerGoalHandle<Action>;
  using State = Nav2Navigation::State;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    client_node_ = std::make_shared<rclcpp::Node>("navigation_test_client");
    server_node_ = std::make_shared<rclcpp::Node>("navigation_test_server");
    server_ = rclcpp_action::create_server<Action>(
        server_node_, "/test_navigate_to_pose",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const Action::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                         : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<ServerHandle>) {
          ++cancellations_;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<ServerHandle> handle) {
          handles_.push_back(handle);
        });
    navigation_ = std::make_unique<Nav2Navigation>(
        client_node_.get(), "/test_navigate_to_pose", 3.0, 3.0);
    pose_.header.frame_id = "map";
    pose_.pose.position.x = 1.25;
    pose_.pose.orientation.w = 1.0;
  }

  bool until(const std::function<bool()> &condition, bool spin_client = true) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
      rclcpp::spin_some(server_node_);
      for (const auto &handle : handles_) {
        if (handle->is_canceling()) {
          handle->canceled(std::make_shared<Action::Result>());
        }
      }
      if (spin_client) {
        rclcpp::spin_some(client_node_);
      }
      navigation_->tick();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  rclcpp::Node::SharedPtr client_node_, server_node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::unique_ptr<Nav2Navigation> navigation_;
  geometry_msgs::msg::PoseStamped pose_;
  std::vector<std::shared_ptr<ServerHandle>> handles_;
  bool reject_ = false;
  int cancellations_ = 0;
};

TEST_F(Nav2NavigationTest, SendsMapPoseAndWaitsForSuccess) {
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return handles_.size() == 1; }));
  EXPECT_EQ(navigation_->state(), State::Navigating);
  EXPECT_EQ(handles_[0]->get_goal()->pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(handles_[0]->get_goal()->pose.pose.position.x, 1.25);
  EXPECT_TRUE(handles_[0]->get_goal()->behavior_tree.empty());
  handles_[0]->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(
      until([this] { return navigation_->state() == State::Succeeded; }));
}

TEST_F(Nav2NavigationTest, RejectedGoalFails) {
  reject_ = true;
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return navigation_->state() == State::Failed; }));
  EXPECT_EQ(navigation_->error(), "rejected");
}

TEST_F(Nav2NavigationTest, AbortedGoalFails) {
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return handles_.size() == 1; }));
  handles_[0]->abort(std::make_shared<Action::Result>());
  ASSERT_TRUE(until([this] { return navigation_->state() == State::Failed; }));
  EXPECT_EQ(navigation_->error(), "aborted");
}

TEST_F(Nav2NavigationTest, MissingServerTimesOut) {
  navigation_ = std::make_unique<Nav2Navigation>(
      client_node_.get(), "/missing_navigation_server", 0.05, 3.0);
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return navigation_->state() == State::Failed; }));
  EXPECT_EQ(navigation_->error(), "server_timeout");
}

TEST_F(Nav2NavigationTest, NavigationTimeoutCancelsAcceptedGoal) {
  navigation_ = std::make_unique<Nav2Navigation>(
      client_node_.get(), "/test_navigate_to_pose", 3.0, 0.2);
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return handles_.size() == 1; }));
  ASSERT_TRUE(until([this] { return cancellations_ == 1; }));
  EXPECT_EQ(navigation_->state(), State::Failed);
  EXPECT_EQ(navigation_->error(), "timeout");
}

TEST_F(Nav2NavigationTest, StopCancelsGoalAndIgnoresCanceledResult) {
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return handles_.size() == 1; }));
  navigation_->cancel();
  ASSERT_TRUE(until([this] { return cancellations_ == 1; }));
  EXPECT_EQ(navigation_->state(), State::Idle);
}

TEST_F(Nav2NavigationTest, LateAcceptanceIsCanceledAfterStop) {
  navigation_->start(pose_);
  // Server accepts, but the client has not processed the goal response yet.
  ASSERT_TRUE(until([this] { return handles_.size() == 1; }, false));
  navigation_->cancel();
  ASSERT_TRUE(until([this] { return cancellations_ == 1; }));
  EXPECT_EQ(navigation_->state(), State::Idle);
}

TEST_F(Nav2NavigationTest, ReplacedGoalCannotCompleteNewOperation) {
  navigation_->start(pose_);
  ASSERT_TRUE(until([this] { return handles_.size() == 1; }, false));
  pose_.pose.position.x = 2.5;
  navigation_->start(pose_);
  ASSERT_TRUE(
      until([this] { return handles_.size() == 2 && cancellations_ == 1; }));
  EXPECT_EQ(navigation_->state(), State::Navigating);
  EXPECT_DOUBLE_EQ(handles_[1]->get_goal()->pose.pose.position.x, 2.5);
  handles_[1]->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(
      until([this] { return navigation_->state() == State::Succeeded; }));
}
}  // namespace
}  // namespace plan_env
