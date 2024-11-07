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

#include "nav2_route_server_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

namespace nav2_route_server_behavior_tree
{

ComputeAndTrackRouteAction::ComputeAndTrackRouteAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ComputeAndTrackRouteAction::on_tick()
{
  getInput("goal", goal_.goal);
  goal_.use_poses = true;

  if (getInput("start_id", goal_.start_id) || getInput("goal_id", goal_.goal_id)) {
    goal_.use_poses = false;
  }

  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputeAndTrackRouteAction::on_success()
{
  setOutput("error_code_id", ActionGoal::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeAndTrackRouteAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeAndTrackRouteAction::on_cancelled()
{
  setOutput("error_code_id", ActionGoal::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeAndTrackRouteAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  if (feedback) {
    setOutput("path", feedback->path);
  }
}

}  // namespace nav2_route_server_behavior_tree

#include <behaviortree_cpp_v3/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<nav2_route_server_behavior_tree::ComputeAndTrackRouteAction>(
      name, "compute_and_track_route", config);
  };

  factory.registerBuilder<nav2_route_server_behavior_tree::ComputeAndTrackRouteAction>(
    "ComputeAndTrackRoute", builder);
}
