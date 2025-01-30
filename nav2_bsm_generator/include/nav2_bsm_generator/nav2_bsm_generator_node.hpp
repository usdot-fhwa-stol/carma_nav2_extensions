/*
 * Copyright (C) 2025 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <j2735_v2x_msgs/msg/transmission_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "nav2_bsm_generator/nav2_bsm_generator_worker.hpp"
#include "nav2_bsm_generator/nav2_bsm_generator_config.hpp"

namespace nav2_bsm_generator
{

  /**
   * \class Nav2BSMGenerator
   * \brief The class responsible for publishing BSM messages
   */
  class Nav2BSMGenerator : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;
    carma_ros2_utils::SubPtr<sensor_msgs::msg::Imu> yaw_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::BSM> bsm_pub_;

    // Timer to run the BSM Generation task
    rclcpp::TimerBase::SharedPtr timer_;

    // Node configuration
    Config config_;

    // Worker class
    std::shared_ptr<Nav2BSMGeneratorWorker> worker;

    // The BSM object that all subscribers make updates to
    carma_v2x_msgs::msg::BSM bsm_;
    std::vector<uint8_t> bsm_message_id_;

    /**
     * \brief Function to fill the BSM message with initial default data
     */ 
    void initializeBSM();

    /**
     * \brief Callback to populate BSM message with longitude, latitude, and elevation data
     * \param msg Latest pose message
     */ 
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with yaw rate data
     * \param msg Latest IMU message
     */ 
    void yawCallback(const sensor_msgs::msg::Imu::UniquePtr msg);

    /**
     * \brief Timer callback, which publishes a BSM
     */ 
    void generateBSM();

  public:
  
    /**
     * \brief Nav2BSMGenerator constructor 
     */
    explicit Nav2BSMGenerator(const rclcpp::NodeOptions &);

    /**
     * \brief Function callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  };

} // namespace nav2_bsm_generator
