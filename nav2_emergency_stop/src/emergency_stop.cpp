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

#include "nav2_emergency_stop/emergency_stop.hpp"

namespace nav2_emergency_stop {

EmergencyStop::EmergencyStop(const rclcpp::NodeOptions &options)
    : rclcpp::Node("emergency_stop", options) {
  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
  node_names_ = get_parameter("node_names").as_string_array();
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      [this](const sensor_msgs::msg::Joy &msg) { on_joy_received(msg); });
  ackermann_publisher_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
          "ackermann_cmd", 1);
}

auto EmergencyStop::on_joy_received(const sensor_msgs::msg::Joy &msg) -> void {
  if (msg.buttons[8]) // Middle Logitech button is the kill switch
  {
    RCLCPP_ERROR(get_logger(), "KILL SWITCH ENABLED");
    for (auto name : node_names_) {
      nav2_util::LifecycleServiceClient lifecycle_client{name,
                                                         shared_from_this()};
      lifecycle_client.change_state(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      lifecycle_client.change_state(
          lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
    }
    ackermann_msgs::msg::AckermannDriveStamped msg;
    ackermann_publisher_->publish(msg);
    rclcpp::shutdown();
  }
}
} // namespace nav2_emergency_stop
