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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/action_node.h>
#include "ros_common.hpp"

class LogAction : public BT::SyncActionNode
{
public:
    LogAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("level"),
            BT::InputPort<std::string>("msg"),
            BT::InputPort<std::string>("arg1"),
            BT::InputPort<std::string>("arg2"),
            BT::InputPort<std::string>("arg3"),
            BT::InputPort<std::string>("arg4")
        };
    }

    virtual BT::NodeStatus tick() override
    {
        std::string level;
        if (!getInput<std::string>("level", level)) {
            throw BT::RuntimeError("missing level");
        }

        std::string msg;
        if (!getInput<std::string>("msg", msg)) {
            throw BT::RuntimeError("missing msg");
        }

        // Optional args to append
        std::string arg;
        for (int i = 1; i <= 4; i++) {
           	std::ostringstream oss;
            oss << "arg" << i;
            if (!getInput<std::string>(oss.str(), arg)) {
                break;
            }
            msg += "| ";
            msg += arg;
        }

        rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

        // FIX - surely a better way to avoid repeating so much code...
        if (level == "fatal") {
            RCLCPP_FATAL(node->get_logger(), "LogAction: %s", msg.c_str());
        } else if (level == "error") {
            RCLCPP_ERROR(node->get_logger(), "LogAction: %s", msg.c_str());
        } else if (level == "warn") {
            RCLCPP_WARN(node->get_logger(), "LogAction: %s", msg.c_str());
        } else if (level == "info") {
            RCLCPP_INFO(node->get_logger(), "LogAction: %s", msg.c_str());
        } else if (level == "debug") {
            RCLCPP_DEBUG(node->get_logger(), "LogAction: %s", msg.c_str());
        } else {
			return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;            
    }
};
