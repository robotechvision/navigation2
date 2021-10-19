// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_smoother/nav2_smoother.hpp"
#include "tf2_ros/create_timer_ros.h"

using SmoothAction = nav2_msgs::action::SmoothPath;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<SmoothAction>;

using namespace std::chrono_literals;

// A smoother for testing the base class

class DummySmoother : public nav2_core::Smoother
{
public:
  DummySmoother()
  : initialized_(false) {}

  ~DummySmoother() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, const std::shared_ptr<tf2_ros::Buffer> &,
    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> &,
    const std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> &) {}

  virtual void cleanup() {}

  virtual void activate() {}

  virtual void deactivate() {}

  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time)
  {
    assert(path.poses.size() == 2);

    auto max_time_ms = max_time.to_chrono<std::chrono::milliseconds>();
    std::this_thread::sleep_for(std::min(max_time_ms, 100ms));

    // place dummy pose in the middle of the path
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x =
      (path.poses.front().pose.position.x + path.poses.back().pose.position.x) / 2;
    pose.pose.position.y =
      (path.poses.front().pose.position.y + path.poses.back().pose.position.y) / 2;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);

    return max_time_ms > 100ms;
  }

private:
  bool initialized_;
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};

class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {
    auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap->metadata.size_x = 100;
    costmap->metadata.size_y = 100;
    costmap->metadata.resolution = 0.1;
    costmap->metadata.origin.position.x = -5.0;
    costmap->metadata.origin.position.y = -5.0;

    costmap->data.resize(costmap->metadata.size_x * costmap->metadata.size_y, 0);
    for (unsigned int i = 0; i < costmap->metadata.size_y; ++i) {
      for (unsigned int j = 20; j < 40; ++j) {
        costmap->data[i * costmap->metadata.size_x + j] = 254;
      }
    }

    setCostmap(costmap);
  }

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

class DummyFootprintSubscriber : public nav2_costmap_2d::FootprintSubscriber
{
public:
  DummyFootprintSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name,
    double footprint_timeout)
  : FootprintSubscriber(node, topic_name, footprint_timeout)
  {
    auto footprint = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    footprint->header.frame_id = "base_link";
    footprint->header.stamp = node->get_clock()->now();
    geometry_msgs::msg::Point32 point;
    point.x = -0.2f;
    point.y = -0.2f;
    footprint->polygon.points.push_back(point);
    point.y = 0.2f;
    footprint->polygon.points.push_back(point);
    point.x = 0.2f;
    point.y = 0.0f;
    footprint->polygon.points.push_back(point);

    setFootprint(footprint);
  }

  void setFootprint(geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    footprint_ = msg;
    footprint_received_ = true;
  }
};

class DummySmootherServer : public nav2_smoother::SmootherServer
{
public:
  DummySmootherServer()
  {
    // don't load default smoothers via pluginlib
    set_parameter(rclcpp::Parameter("smoother_plugins", std::vector<std::string>()));
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state)
  {
    auto result = SmootherServer::on_configure(state);
    assert(
      result == nav2_util::CallbackReturn::SUCCESS &&
      smoother_ids_.empty() && smoother_types_.empty() && smoothers_.empty());

    // Create dummy subscribers and collision checker
    auto node = shared_from_this();
    costmap_sub_ =
      std::make_shared<DummyCostmapSubscriber>(
      node, "costmap_topic");
    footprint_sub_ =
      std::make_shared<DummyFootprintSubscriber>(
      node, "footprint_topic", 10.0);
    collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_, *tf_,
      node->get_name(),
      "base_link", "base_link");  // global frame = robot frame to avoid tf lookup

    // Create dummy smoother
    smoother_ids_.push_back("DummySmoothPath");
    smoother_types_.push_back("DummySmoother");
    nav2_core::Smoother::Ptr smoother = std::make_shared<DummySmoother>();
    smoother->configure(
      shared_from_this(), smoother_ids_[0], tf_, costmap_sub_,
      footprint_sub_);
    smoothers_.insert({smoother_ids_[0], smoother});

    return result;
  }
};

// Define a test class to hold the context for the tests

class SmootherTest : public ::testing::Test
{
protected:
  SmootherTest() {SetUp();}
  ~SmootherTest() {}

  void SetUp()
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleSmootherTestNode", rclcpp::NodeOptions());

    smoother_server_ = std::make_shared<DummySmootherServer>();
    smoother_server_->configure();
    smoother_server_->activate();

    client_ = rclcpp_action::create_client<SmoothAction>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_graph_interface(),
      node_lifecycle_->get_node_logging_interface(),
      node_lifecycle_->get_node_waitables_interface(), "smooth_path");
    std::cout << "Setup complete." << std::endl;
  }

  void TearDown() override {}

  bool sendGoal(
    std::string smoother_id, double x_start, double y_start, double x_goal,
    double y_goal, std::chrono::milliseconds max_time, bool check_for_collisions)
  {
    if (!client_->wait_for_action_server(4s)) {
      std::cout << "Server not up" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.w = 1.0;

    auto goal = SmoothAction::Goal();
    goal.smoother_id = smoother_id;
    pose.pose.position.x = x_start;
    pose.pose.position.y = y_start;
    goal.path.poses.push_back(pose);
    pose.pose.position.x = x_goal;
    pose.pose.position.y = y_goal;
    goal.path.poses.push_back(pose);
    goal.check_for_collisions = check_for_collisions;
    goal.max_smoothing_duration = rclcpp::Duration(max_time);

    auto future_goal = client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(node_lifecycle_, future_goal) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "failed sending goal" << std::endl;
      // failed sending the goal
      return false;
    }

    goal_handle_ = future_goal.get();

    if (!goal_handle_) {
      std::cout << "goal was rejected" << std::endl;
      // goal was rejected by the action server
      return false;
    }

    return true;
  }

  ClientGoalHandle::WrappedResult getResult()
  {
    std::cout << "Getting async result..." << std::endl;
    auto future_result = client_->async_get_result(goal_handle_);
    std::cout << "Waiting on future..." << std::endl;
    rclcpp::spin_until_future_complete(node_lifecycle_, future_result);
    std::cout << "future received!" << std::endl;
    return future_result.get();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<DummySmootherServer> smoother_server_;
  std::shared_ptr<rclcpp_action::Client<SmoothAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<SmoothAction>> goal_handle_;
};

// Define the tests

TEST_F(SmootherTest, testingSuccess)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", 0.0, 0.0, 1.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->path.poses.size(), (std::size_t)3);
  EXPECT_TRUE(result.result->was_completed);
  SUCCEED();
}

TEST_F(SmootherTest, testingFailureOnInvalidPlugin)
{
  ASSERT_TRUE(sendGoal("InvalidPlugin", 0.0, 0.0, 1.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  SUCCEED();
}

TEST_F(SmootherTest, testingSuccessOnEmptyPlugin)
{
  ASSERT_TRUE(sendGoal("", 0.0, 0.0, 1.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  SUCCEED();
}

TEST_F(SmootherTest, testingIncomplete)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", 0.0, 0.0, 1.0, 0.0, 50ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->was_completed);
  SUCCEED();
}

TEST_F(SmootherTest, testingFailureOnCollision)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", -4.0, 0.0, 0.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  SUCCEED();
}

TEST_F(SmootherTest, testingCollisionCheckDisabled)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", -4.0, 0.0, 0.0, 0.0, 500ms, false));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
