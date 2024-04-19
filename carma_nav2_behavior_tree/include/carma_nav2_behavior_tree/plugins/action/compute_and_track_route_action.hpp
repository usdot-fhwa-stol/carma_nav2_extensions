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

#ifndef CARMA_NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_AND_TRACK_ROUTE_ACTION_HPP_
#define CARMA_NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_AND_TRACK_ROUTE_ACTION_HPP_

#include <string>

#include <nav_msgs/msg/path.h>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_msgs/action/compute_and_track_route.hpp>

namespace carma_nav2_behavior_tree
{
class ComputeAndTrackRouteAction
: public nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputeAndTrackRoute>
{
  using Action = nav2_msgs::action::ComputeAndTrackRoute;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  ComputeAndTrackRouteAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::uint16_t>("start_id", "Start node in graph"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("start", "Start pose of the route"),
      BT::InputPort<std::uint16_t>("goal_id", "Destination node in graph"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination pose of the route"),
      BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputeAndTrackRoute node"),
      BT::OutputPort<ActionResult::_error_code_type>(
        "error_code_id", "The compute path through poses error code"),
    });
  }
};
}  // namespace carma_nav2_behavior_tree

#endif  // CARMA_NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_AND_TRACK_ROUTE_ACTION_HPP_
