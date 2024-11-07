// Copyright 2024 Leidos
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
// limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_route_server_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

class ComputeAndTrackRouteActionServer
: public TestActionServer<nav2_msgs::action::ComputeAndTrackRoute>
{
public:
  ComputeAndTrackRouteActionServer() : TestActionServer("compute_and_track_route") {}

protected:
  void execute(const typename std::shared_ptr<
               rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputeAndTrackRoute>>
                 goal_handle) override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::ComputeAndTrackRoute::Result>();
    if (std::isnan(goal->goal.pose.position.x)) {
      result->error_code = 1;
      goal_handle->abort(result);
      return;
    } else if (goal->goal.pose.position.x > 100.0) {
      while (!goal_handle->is_canceling()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
      }
      return;
    }
    goal_handle->succeed(result);
  }
};

class ComputeAndTrackRouteActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("compute_and_track_route_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set("initial_pose_received", false);
    client_ = rclcpp_action::create_client<nav2_msgs::action::ComputeAndTrackRoute>(
      node_, "compute_and_track_route");

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_route_server_behavior_tree::ComputeAndTrackRouteAction>(
        name, "compute_and_track_route", config);
    };

    factory_->registerBuilder<nav2_route_server_behavior_tree::ComputeAndTrackRouteAction>(
      "ComputeAndTrackRoute", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
    client_.reset();
  }

  void TearDown() override { tree_.reset(); }

  static std::shared_ptr<ComputeAndTrackRouteActionServer> action_server_;
  static std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputeAndTrackRoute>> client_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ComputeAndTrackRouteActionTestFixture::node_ = nullptr;
std::shared_ptr<ComputeAndTrackRouteActionServer>
  ComputeAndTrackRouteActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * ComputeAndTrackRouteActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ComputeAndTrackRouteActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ComputeAndTrackRouteActionTestFixture::tree_ = nullptr;
std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputeAndTrackRoute>>
  ComputeAndTrackRouteActionTestFixture::client_ = nullptr;

TEST_F(ComputeAndTrackRouteActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
        <root BTCPP_format="4">
          <BehaviorTree ID="MainTree">
              <ComputeAndTrackRoute goal="{goal}"/>
          </BehaviorTree>
        </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  // tick until node succeeds
  uint16_t iter = 0;  // safety to prevent infinite loop
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS && iter < 500) {
    tree_->rootNode()->executeTick();
    ++iter;
  }

  // goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal, goal);

  // halt node so another goal can be sent
  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal
  goal.pose.position.x = -2.5;
  goal.pose.orientation.x = 1.0;
  config_->blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);

  iter = 0;
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS && iter < 500) {
    tree_->rootNode()->executeTick();
    ++iter;
  }

  EXPECT_EQ(action_server_->getCurrentGoal()->goal, goal);
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

TEST_F(ComputeAndTrackRouteActionTestFixture, test_cancel)
{
  // create tree
  std::string xml_txt =
    R"(
        <root BTCPP_format="4">
          <BehaviorTree ID="MainTree">
              <ComputeAndTrackRoute goal="{goal}"/>
          </BehaviorTree>
        </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1000.0;
  config_->blackboard->set("goal", goal);
  // Send a request to cancel the goal
  client_->async_cancel_all_goals();

  // tick until node succeeds
  uint16_t iter = 0;  // safety to prevent infinite loop
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
         tree_->rootNode()->status() != BT::NodeStatus::FAILURE && iter < 500) {
    tree_->rootNode()->executeTick();
    ++iter;
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(action_server_->isGoalCancelled());
}

TEST_F(ComputeAndTrackRouteActionTestFixture, test_abort)
{
  // create tree
  std::string xml_txt =
    R"(
        <root BTCPP_format="4">
          <BehaviorTree ID="MainTree">
              <ComputeAndTrackRoute goal="{goal}"/>
          </BehaviorTree>
        </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  config_->blackboard->set("goal", goal);

  // tick until node fails
  uint16_t iter = 0;  // safety to prevent infinite loop
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
         tree_->rootNode()->status() != BT::NodeStatus::FAILURE && iter < 500) {
    tree_->rootNode()->executeTick();
    ++iter;
  }

  // the goal should have been aborted due to NaN in input
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  ComputeAndTrackRouteActionTestFixture::action_server_ =
    std::make_shared<ComputeAndTrackRouteActionServer>();

  std::thread server_thread(
    []() { rclcpp::spin(ComputeAndTrackRouteActionTestFixture::action_server_); });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
