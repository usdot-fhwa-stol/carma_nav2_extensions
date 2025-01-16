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

#ifndef NAV2_PORT_DRAYAGE_DEMO__PORT_DRAYAGE_DEMO_HPP_
#define NAV2_PORT_DRAYAGE_DEMO__PORT_DRAYAGE_DEMO_HPP_

#include <std_srvs/srv/trigger.hpp>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <memory>
#include <nav2_msgs/action/compute_and_track_route.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

namespace nav2_port_drayage_demo
{

class OperationID
{
public:
  /**
    * \brief Enum containing possible operation IDs used to define destinations for port drayage.
    */
  enum Operation
  {
    PICKUP,
    DROPOFF,
    ENTER_STAGING_AREA,
    EXIT_STAGING_AREA,
    ENTER_PORT,
    EXIT_PORT,
    PORT_CHECKPOINT,
    HOLDING_AREA,
    DEFAULT_OPERATION,
  };

  /**
    * \brief Standard constructor for OperationID
    * \param op Operation enum associated with this object.
    */
  explicit OperationID(enum Operation op) : operation_enum_(op) {}

  /**
    * \brief Create OperationID using a string
    * \param op_str String to used to create the Operation
    */
  explicit OperationID(std::string op_str);

  /**
    * \brief Getter function to obtain the Operation enum associated with this object.
    * \return Operation enum associated with this object.
    */
  OperationID::Operation getOperationID() const;

  /**
    * \brief Function to convert a string to Operation
    * \param op_str String to convert to an Operation
    */
  OperationID::Operation stringToOperation(std::string op_str) const;

  /**
    * \brief Function to convert this object's 'operation_enum_' to a human-readable string.
    * \return A human-readable string representing this object's 'operation_enum_'.
    */
  std::string operationToString() const;

  /**
    * \brief Stream operator for this object.
    */
  friend std::ostream & operator<<(std::ostream & output, const OperationID & oid)
  {
    return output << oid.operationToString();
  }

  /**
    * \brief Overloaded == operator for comparision with String objects.
    */
  friend bool operator==(const std::string & lhs, const OperationID & rhs)
  {
    return lhs == rhs.operationToString();
  }

private:
  // Data member containing this object's Operation enum value
  const Operation operation_enum_ = Operation::DEFAULT_OPERATION;
};

/**
  * \brief Convenience struct for storing all data contained in a received MobilityOperation message's
  * strategy_params field with strategy "carma/port_drayage"
  */
struct PortDrayageMobilityOperationMsg
{
  std::string cargo_id;
  std::shared_ptr<OperationID> operation;
  std::string current_action_id;  // Identifier for the action this message is related to
  double dest_longitude;          // Destination longitude for the CDA1Tenth vehicle
  double dest_latitude;           // Destination latitude for the CDA1Tenth vehicle
  double start_longitude;         // Starting longitude of the CDA1Tenth vehicle
  double start_latitude;          // Starting latitude of the CDA1Tenth vehicle
  PortDrayageMobilityOperationMsg()
  {
    operation =
      std::shared_ptr<OperationID>(new OperationID{OperationID::Operation::DEFAULT_OPERATION});
  }
};

/**
 * Port Drayage implementation integrated with the Navigation2 autonomy stack
 */
class PortDrayageDemo : public rclcpp_lifecycle::LifecycleNode
{
public:

  /**
   * \brief PortDrayageDemo constructor
   */
  explicit PortDrayageDemo(const rclcpp::NodeOptions & options);

  /**
   * \brief Callback for incoming mobility operation messages
   * \param msg Incoming mobility operation message
   */
  auto on_mobility_operation_received(const carma_v2x_msgs::msg::MobilityOperation & msg) -> void;

  /**
   * \brief Service handler to send an ack when the truck enters the port
   * \param request Empty message to trigger service
   * \param response String response
   */
  void handle_entrance_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * \brief Callback triggered after generating a route, to send the route to a controller
   * \param feedback The route and path from ComputeAndTrackRoute
   */
  auto route_feedback_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputeAndTrackRoute>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Feedback> feedback) -> void;

  /**
   * \brief Callback triggered after a port drayage action is completed to publish an ack
   * \param result The result of the action
   */
  auto route_result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputeAndTrackRoute>::WrappedResult &
      result) -> void;
  
  /**
   * \brief Callback triggered after a port drayage action is completed to publish an ack
   * \param result The result of the action
   */
  auto follow_path_result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult &
      result) -> void;

  /**
   * \brief Callback that stores the vehicle's odometry
   * \param msg The odometry to store
   */
  auto on_odometry_received(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) -> void;

  /**
   * \brief Callback that contains status information on Nav2 goals sent from Rviz
   * \param msg Goal status
   */
  auto on_rviz_goal_status_received(const action_msgs::msg::GoalStatusArray & msg) -> void;

  /**
   * \brief Helper function to compose the ack published in on_result_received
   */
  auto compose_arrival_message() -> carma_v2x_msgs::msg::MobilityOperation;

  /**
   * \brief Helper function that creates a PortDrayageMobilityOperationMsg from a ROS message
   * \param msg The mobility operation message to convert to a struct and store in previous_mobility_operation_msg_
   */
  auto extract_port_drayage_message(const carma_v2x_msgs::msg::MobilityOperation & msg) -> bool;

  ////
  // Overrides
  ////
  auto on_configure(const rclcpp_lifecycle::State & state) -> nav2_util::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & state) -> nav2_util::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & state) -> nav2_util::CallbackReturn override;

  ////
  // Accessors
  ////
  auto set_cmv_id(std::string cmv_id) -> void { cmv_id_ = cmv_id; }

  auto get_cmv_id() -> std::string { return cmv_id_; }

  auto get_cargo_id() -> std::string { return cargo_id_; }

private:
  ////
  // Pubs & Subs
  ////
  rclcpp::Subscription<carma_v2x_msgs::msg::MobilityOperation>::SharedPtr
    mobility_operation_subscription_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    odometry_subscription_{nullptr};

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
    rviz_action_subscription_{nullptr};

  rclcpp::Publisher<carma_v2x_msgs::msg::MobilityOperation>::SharedPtr
    mobility_operation_publisher_{nullptr};

  ////
  // Action Client
  ////
  rclcpp_action::Client<nav2_msgs::action::ComputeAndTrackRoute>::SharedPtr route_client_{nullptr};
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_{nullptr};
  nav_msgs::msg::Path computed_path_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

  // Current odometry
  geometry_msgs::msg::PoseWithCovarianceStamped current_odometry_;
  // Vehicle identifier
  std::string cmv_id_;
  // Delay between receiving and processing goal message
  int message_processing_delay_;
  // Cargo identified
  std::string cargo_id_;
  // Clock
  rclcpp::Clock::SharedPtr clock_;
  // Previously received Mobility Operation message
  PortDrayageMobilityOperationMsg previous_mobility_operation_msg_;
};
}  // namespace nav2_port_drayage_demo

#endif  // NAV2_PORT_DRAYAGE_DEMO__PORT_DRAYAGE_DEMO_HPP_
