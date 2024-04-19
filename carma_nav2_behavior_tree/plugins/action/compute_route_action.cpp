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

#include "carma_nav2_behavior_tree/plugins/action/compute_route_action.hpp"

namespace carma_nav2_behavior_tree
{

ComputeRouteAction::ComputeRouteAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ComputeRouteAction::on_tick()
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

BT::NodeStatus ComputeRouteAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("error_code_id", ActionGoal::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeRouteAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeRouteAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  setOutput("error_code_id", ActionGoal::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeRouteAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace carma_nav2_behavior_tree

#include <behaviortree_cpp_v3/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<carma_nav2_behavior_tree::ComputeRouteAction>(
      name, "compute_route", config);
  };

  factory.registerBuilder<carma_nav2_behavior_tree::ComputeRouteAction>("ComputeRoute", builder);
}
