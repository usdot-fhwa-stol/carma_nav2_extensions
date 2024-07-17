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
#include "carma_nav2_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

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
    auto result = std::make_shared<nav2_msgs::action::ComputeAndTrackRoute::Result>();
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
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));
    config_->blackboard->set("initial_pose_received", false);

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<carma_nav2_behavior_tree::ComputeAndTrackRouteAction>(
        name, "compute_and_track_route", config);
    };

    factory_->registerBuilder<carma_nav2_behavior_tree::ComputeAndTrackRouteAction>(
      "ComputeAndTrackRoute", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override { tree_.reset(); }

  static std::shared_ptr<ComputeAndTrackRouteActionServer> action_server_;

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
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
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

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(action_server_->getCurrentGoal()->goal, goal);
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
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
