/*
Copyright 2021 Scott Horton

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
#include <behaviortree_cpp_v3/action_node.h>

class GetRobotPoseAction: public BT::SyncActionNode
{
    public:
		GetRobotPoseAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<std::string>("pose")};
        }

        virtual BT::NodeStatus tick() override
        {
            RobotStatus* robot_status = RobotStatus::GetInstance();
            if (!robot_status) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cannot get robot status for obtaining pose");
                return BT::NodeStatus::FAILURE;
            }
            
            double x, y, z, yaw;
            auto got_pose = robot_status->GetPose(x, y, z, yaw);
            if (!got_pose) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Robot pose not available.");
                return BT::NodeStatus::FAILURE;
            }

            std::stringstream ss;
            ss << x << "," << y << "," << yaw;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot pose: %s", ss.str().c_str());

			setOutput("pose", ss.str());
            return BT::NodeStatus::SUCCESS;
        }
};
