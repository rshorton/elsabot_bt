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

#include <stdio.h>
#include <string>
#include <regex>

#include <nlohmann/json.hpp>

#include <behaviortree_cpp/action_node.h>

#include "ros_common.hpp"

// Use this action to parse params in a toolcall and save to the black board

class ToolCallParseParamToBBAction : public BT::StatefulActionNode
{
public:
	  ToolCallParseParamToBBAction(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {
        node_ = ROSCommon::GetInstance()->GetNode();        
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("tc_args"),
                BT::InputPort<std::string>("obj_name"),
                BT::OutputPort<std::string>("output")};
    }

    BT::NodeStatus onStart() {
        std::string tc_args;
        if (!getInput<std::string>("tc_args", tc_args)) {
      		throw BT::RuntimeError("missing tc_args");
        }

        std::string obj_name;
        if (!getInput<std::string>("obj_name", obj_name)) {
      		throw BT::RuntimeError("missing obj_name");
        }

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(tc_args);
        } catch (nlohmann::json::parse_error& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to parse tc args %s, at: %ld", tc_args.c_str(), ex.byte);
            return BT::NodeStatus::FAILURE;
        }

        if (!j.contains(obj_name)) {
            RCLCPP_ERROR(node_->get_logger(), "Object name %s not in tool call args: %s", obj_name.c_str(), tc_args.c_str());
            return BT::NodeStatus::FAILURE;
        }
        setOutput("output", j[obj_name].get<std::string>());
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() {
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() {
    }

private:
    rclcpp::Node::SharedPtr node_;
};
