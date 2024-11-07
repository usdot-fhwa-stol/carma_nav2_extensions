# `nav2_route_server_behavior_tree`

This package contains additional behavior tree (BT) nodes to supplement those in the `nav2_behavior_tree`
package. Currently, the `nav2_behavior_tree` package does not have BT Action nodes to interface with the
Route Server.

See [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html) and
[Writing a New Behavior Tree Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html)
for more information about BTs and how they integrate with Nav2.

## BT Action nodes

- **ComputeRoute Action node**: sends a `ComputeRoute` ROS 2 action request to the Route Server and stores
  the result in a Behavior Tree Blackboard variable.
- **ComputeAndTrackRouteAction node**: sends a `ComputeAndTrackRoute` ROS 2 action request and stores/updates
  the Route Server's generated dense path (received through a ROS 2 action feedback message) in a Behavior
  Tree Blackboard variable.
