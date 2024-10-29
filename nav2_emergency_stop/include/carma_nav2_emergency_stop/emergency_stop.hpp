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

#ifndef NAV2_EMERGENCY_STOP__EMERGENCY_STOP_HPP_
#define NAV2_EMERGENCY_STOP__EMERGENCY_STOP_HPP_

#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/lifecycle_service_client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace nav2_emergency_stop
{

class EmergencyStop : public rclcpp::Node
{
public:
  explicit EmergencyStop(const rclcpp::NodeOptions & options);

  auto on_joy_received(const sensor_msgs::msg::Joy & msg) -> void;

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
    joy_subscription_{nullptr};

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    ackermann_publisher_{nullptr};

  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;
};
}  // namespace nav2_emergency_stop

#endif  // NAV2_EMERGENCY_STOP__EMERGENCY_STOP_HPP_
