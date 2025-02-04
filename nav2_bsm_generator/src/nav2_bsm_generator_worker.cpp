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

#include "nav2_bsm_generator/nav2_bsm_generator_worker.hpp"
#include <random>

namespace nav2_bsm_generator
{
    
    Nav2BSMGeneratorWorker::Nav2BSMGeneratorWorker() {}
    
    uint8_t Nav2BSMGeneratorWorker::getNextMsgCount()
    {
        uint8_t old_msg_count = msg_count_;
        msg_count_++;
        if(msg_count_ == 128)
        {
            msg_count_ = 0;
        }
        return old_msg_count;
    }

    std::vector<uint8_t> Nav2BSMGeneratorWorker::getMsgId(const rclcpp::Time now, double secs)
    {
        // need to change ID every designated period
        rclcpp::Duration id_timeout(int32_t(secs * 1e9), 0);

        generator_.seed(std::random_device{}()); // guarantee randomness
        std::uniform_int_distribution<int> dis(0,INT_MAX);
        
        std::vector<uint8_t> id(4);

        if (first_msg_id_) {
            last_id_generation_time_ = now;
            first_msg_id_ = false;
            random_id_ = dis(generator_);
        }
        
        if(now - last_id_generation_time_ >= id_timeout)
        {
            random_id_ = dis(generator_);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("nav2_bsm_generator"), "Newly generated random id: " << random_id_);
            last_id_generation_time_ = now;
        }

        for(int i = 0; i < id.size(); ++i)
        {
            id[i] = random_id_ >> (8 * i);
        }
        return id;
    }

    uint16_t Nav2BSMGeneratorWorker::getSecMark(const rclcpp::Time now)
    {
        return static_cast<uint16_t>((now.nanoseconds() / 1000000) % 60000);
    }

    float Nav2BSMGeneratorWorker::getSpeedInRange(const double speed)
    {
        return static_cast<float>(std::max(std::min(speed, 163.8), 0.0));
    }

    float Nav2BSMGeneratorWorker::getSteerWheelAngleInRange(const double angle)
    {
        return static_cast<float>(std::max(std::min(angle * 57.2958, 189.0), -189.0));
    }

    float Nav2BSMGeneratorWorker::getLongAccelInRange(const float accel)
    {
        return std::max(std::min(accel, 20.0f), -20.0f);
    }

    float Nav2BSMGeneratorWorker::getYawRateInRange(const double yaw_rate)
    {
        return static_cast<float>(std::max(std::min(yaw_rate, 327.67), -327.67));
    }

    uint8_t Nav2BSMGeneratorWorker::getBrakeAppliedStatus(const double brake)
    {
        return brake >= 0.05 ? 0b1111 : 0;
    }

    float Nav2BSMGeneratorWorker::getHeadingInRange(const float heading)
    {
        return std::max(std::min(heading, 359.9875f), 0.0f);
    }
} // namespace nav2_bsm_generator