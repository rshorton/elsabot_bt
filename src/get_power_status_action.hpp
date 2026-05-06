/*
Copyright 2026 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#pragma once

#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/action_node.h>

#include "robot_status.hpp"

class GetPowerStatusAction: public BT::SyncActionNode
{
    public:
		GetPowerStatusAction(const std::string& name, const BT::NodeConfig& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<float>("voltage"),
                    BT::OutputPort<float>("charge_pct")};
        }

        virtual BT::NodeStatus tick() override
        {
            RobotStatus* robot_status = RobotStatus::GetInstance();
            if (!robot_status) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cannot get robot status for obtaining power status");
                return BT::NodeStatus::FAILURE;
            }

            sensor_msgs::msg::BatteryState status;
            if (!robot_status->GetBatteryStatus(status)) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Battery status not available");
                return BT::NodeStatus::FAILURE;
            }

            setOutput("voltage", status.voltage);
            setOutput("charge_pct", status.percentage*100.0f);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Power status: v: %f, charge: %f", status.voltage, status.percentage);

            return BT::NodeStatus::SUCCESS;
        }
};
