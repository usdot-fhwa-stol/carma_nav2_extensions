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

#include "nav2_port_drayage_demo/port_drayage_demo.hpp"

namespace nav2_port_drayage_demo
{

OperationID::OperationID(std::string op_str) : operation_enum_(stringToOperation(op_str)) {}

OperationID::Operation OperationID::getOperationID() const { return operation_enum_; }

OperationID::Operation OperationID::stringToOperation(std::string op_str) const
{
  if (op_str == "PICKUP")
    return Operation::PICKUP;
  else if (op_str == "DROPOFF")
    return Operation::DROPOFF;
  else if (op_str == "ENTER_STAGING_AREA")
    return Operation::ENTER_STAGING_AREA;
  else if (op_str == "EXIT_STAGING_AREA")
    return Operation::EXIT_STAGING_AREA;
  else if (op_str == "ENTER_PORT")
    return Operation::ENTER_PORT;
  else if (op_str == "EXIT_PORT")
    return Operation::EXIT_PORT;
  else if (op_str == "PORT_CHECKPOINT")
    return Operation::PORT_CHECKPOINT;
  else if (op_str == "HOLDING_AREA")
    return Operation::HOLDING_AREA;
  else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("OperationID"), "Received unknown Operation " << op_str);
    return Operation::DEFAULT_OPERATION;
  }
}

std::string OperationID::operationToString() const
{
  // Convert operation enum into a human-readable string
  switch (operation_enum_) {
    case Operation::PICKUP:
      return "PICKUP";
    case Operation::DROPOFF:
      return "DROPOFF";
    case Operation::ENTER_STAGING_AREA:
      return "ENTER_STAGING_AREA";
    case Operation::EXIT_STAGING_AREA:
      return "EXIT_STAGING_AREA";
    case Operation::ENTER_PORT:
      return "ENTER_PORT";
    case Operation::EXIT_PORT:
      return "EXIT_PORT";
    case Operation::PORT_CHECKPOINT:
      return "PORT_CHECKPOINT";
    case Operation::HOLDING_AREA:
      return "HOLDING_AREA";
    default:
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("OperationID"),
        "Conversion of an unsupported operation enum value to a string.");
      return "DEFAULT_OPERATION";
  }
}

PortDrayageDemo::PortDrayageDemo(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("port_drayage_demo", options)
{
  declare_parameter("cmv_id", rclcpp::ParameterValue(std::string("")));
  declare_parameter("message_processing_delay", 0);
}

auto PortDrayageDemo::on_configure(const rclcpp_lifecycle::State & /* state */)
  -> nav2_util::CallbackReturn
{
  get_parameter("cmv_id", cmv_id_);
  get_parameter("message_processing_delay", message_processing_delay_);

  clock_ = get_clock();

  service_ = this->create_service<std_srvs::srv::Trigger>(
        "entrance_reached", std::bind(&PortDrayageDemo::handle_entrance_trigger, this, std::placeholders::_1, std::placeholders::_2));

  route_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputeAndTrackRoute>(this, "compute_and_track_route");
  follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "follow_path");

  mobility_operation_subscription_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>(
    "incoming_mobility_operation", 1, [this](const carma_v2x_msgs::msg::MobilityOperation & msg) {
      on_mobility_operation_received(msg);
    });

  odometry_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
      on_odometry_received(msg);
    });

  rviz_action_subscription_ = create_subscription<action_msgs::msg::GoalStatusArray>(
    "/navigate_to_pose/_action/status", 1, [this](const action_msgs::msg::GoalStatusArray & msg) {
      on_rviz_goal_status_received(msg);
    });

  mobility_operation_publisher_ =
    create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 1);

  return nav2_util::CallbackReturn::SUCCESS;
}

auto PortDrayageDemo::on_activate(const rclcpp_lifecycle::State & /* state */)
  -> nav2_util::CallbackReturn
{
  return nav2_util::CallbackReturn::SUCCESS;
}

auto PortDrayageDemo::on_deactivate(const rclcpp_lifecycle::State & /* state */)
  -> nav2_util::CallbackReturn
{
  return nav2_util::CallbackReturn::SUCCESS;
}

auto PortDrayageDemo::on_mobility_operation_received(
  const carma_v2x_msgs::msg::MobilityOperation & msg) -> void
{
  if (!extract_port_drayage_message(msg)) return;
  rclcpp::sleep_for(std::chrono::seconds(message_processing_delay_));
  nav2_msgs::action::ComputeAndTrackRoute::Goal goal;

  geometry_msgs::msg::PoseStamped pose;
  goal.start_id = 1u;
  goal.start.header.frame_id = "map";
  goal.start.header.stamp = this->now();
  goal.start.pose.position.x = current_odometry_.pose.pose.position.x;
  goal.start.pose.position.y = current_odometry_.pose.pose.position.y;
  goal.goal.header.frame_id = "map";
  goal.goal.header.stamp = this->now();
  goal.goal.pose.position.x = previous_mobility_operation_msg_.dest_longitude;
  goal.goal.pose.position.y = previous_mobility_operation_msg_.dest_latitude;
  goal.goal_id = 2u;
  goal.use_start = false;
  goal.use_poses = true;
  
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::ComputeAndTrackRoute>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(
    &PortDrayageDemo::route_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
    &PortDrayageDemo::route_result_callback, this, std::placeholders::_1);
  route_client_->async_send_goal(goal, send_goal_options);
}

void PortDrayageDemo::route_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputeAndTrackRoute>::SharedPtr,
                               const std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Received feedback with path containing %zu poses", feedback->path.poses.size());

  if (feedback->path.poses.size() > 0 && !goal_sent_)  {
    
    auto follow_path_goal = nav2_msgs::action::FollowPath::Goal();
    follow_path_goal.path = feedback->path;
    follow_path_goal.controller_id = "";

    follow_path_client_->async_send_goal(follow_path_goal);
    goal_sent_ = true;
  }
}

void PortDrayageDemo::handle_entrance_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Unused
  if (publish_mom_)
  {
    carma_v2x_msgs::msg::MobilityOperation ack = compose_arrival_message();
    mobility_operation_publisher_->publish(std::move(ack));
    publish_mom_ = false;
    last_arrival_location_ = current_odometry_;
    response->success = true;
    response->message = "Publishing entrance arrival message.";
  } else {
    response->success = false;
    response->message = "Arrival message already published, ignoring...";
  }
  return;
}

void PortDrayageDemo::route_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputeAndTrackRoute>::WrappedResult & result)
{
  goal_sent_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      // Create arrival message
      carma_v2x_msgs::msg::MobilityOperation result = compose_arrival_message();
      mobility_operation_publisher_->publish(std::move(result));
      return;
    }
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "An unknown error occurred");
      return;
  }
}

auto PortDrayageDemo::on_odometry_received(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg) -> void
{
  current_odometry_ = msg;
  // Check if truck has travelled 1 meter since publishing MOM arrival and clear flag if so
  if (std::sqrt(
        std::pow(current_odometry_.pose.pose.position.x - last_arrival_location_.pose.pose.position.x, 2)
        + std::pow(current_odometry_.pose.pose.position.y - last_arrival_location_.pose.pose.position.y, 2)) > 1.0)
  {
    publish_mom_ = true;
  }
}

auto PortDrayageDemo::on_rviz_goal_status_received(
  const action_msgs::msg::GoalStatusArray & msg) -> void
{
  const auto & goal = msg.status_list[msg.status_list.size() - 1];
  if (goal.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    // Create arrival message
    carma_v2x_msgs::msg::MobilityOperation result = compose_arrival_message();
    mobility_operation_publisher_->publish(std::move(result));
    rviz_action_subscription_.reset(); 
  }
}

auto PortDrayageDemo::extract_port_drayage_message(
  const carma_v2x_msgs::msg::MobilityOperation & msg) -> bool
{
  try {
    const auto strategy_params_json = nlohmann::json::parse(msg.strategy_params);
    const auto cmv_id{strategy_params_json["cmv_id"].template get<std::string>()};
    if (cmv_id != cmv_id_) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring received port drayage MobilityOperation message intended for cmv_id %s",
        cmv_id.c_str());
      return false;
    } else if (msg.strategy != "carma/port_drayage") {
      RCLCPP_WARN(
        get_logger(), "Ignoring received MobilityOperation message with strategy %s",
        msg.strategy.c_str());
      return false;
    }

    previous_mobility_operation_msg_.start_longitude = current_odometry_.pose.pose.position.x;
    previous_mobility_operation_msg_.start_latitude = current_odometry_.pose.pose.position.y;
    previous_mobility_operation_msg_.dest_longitude =
      std::stod(strategy_params_json["destination"]["longitude"].template get<std::string>());
    previous_mobility_operation_msg_.dest_latitude =
      std::stod(strategy_params_json["destination"]["latitude"].template get<std::string>());
    previous_mobility_operation_msg_.operation = std::shared_ptr<OperationID>(
      new OperationID(strategy_params_json["operation"].template get<std::string>()));
    previous_mobility_operation_msg_.cargo_id =
      strategy_params_json["cargo_id"].template get<std::string>();
    previous_mobility_operation_msg_.current_action_id =
      strategy_params_json["action_id"].template get<std::string>();
  } catch (const nlohmann::json::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Could not process MobilityOperation message: JSON error: " << e.what());
    return false;
  }
  return true;
}

auto PortDrayageDemo::compose_arrival_message() -> carma_v2x_msgs::msg::MobilityOperation
{
  carma_v2x_msgs::msg::MobilityOperation result;
  // Populate message header
  result.m_header.sender_id = cmv_id_;
  result.m_header.recipient_id = "";
  result.m_header.plan_id = "";
  result.m_header.timestamp = clock_->now().nanoseconds() / 1E6;  // epoch to uint64 milliseconds
  // Set strategy
  result.strategy = "carma/port_drayage";
  // Set JSON fields
  nlohmann::json mobility_operation_json, location_json;
  location_json["longitude"] = current_odometry_.pose.pose.position.x;
  location_json["latitude"] = current_odometry_.pose.pose.position.y;
  mobility_operation_json["location"] = location_json;
  mobility_operation_json["cmv_id"] = cmv_id_;
  mobility_operation_json["operation"] =
    previous_mobility_operation_msg_.operation->operationToString();
  mobility_operation_json["action_id"] = previous_mobility_operation_msg_.current_action_id;
  // Update Cargo ID and set JSON field
  if (
    previous_mobility_operation_msg_.operation->getOperationID() ==
    OperationID::Operation::PICKUP) {
    cargo_id_ = previous_mobility_operation_msg_.cargo_id;
    mobility_operation_json["cargo_id"] = cargo_id_;
  } else if (
    previous_mobility_operation_msg_.operation->getOperationID() ==
    OperationID::Operation::DROPOFF) {
    cargo_id_ = "";
  } else if (cargo_id_ != "") {
    mobility_operation_json["cargo_id"] = cargo_id_;
  }
  mobility_operation_json["cargo"] = cargo_id_ != "";
  result.strategy_params = mobility_operation_json.dump();
  return result;
}
}  // namespace nav2_port_drayage_demo
