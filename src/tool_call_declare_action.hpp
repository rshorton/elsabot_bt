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

#include <behaviortree_cpp/action_node.h>

#include "ros_common.hpp"
#include "tool_call_data.hpp"

// Use this action to declare an AI tool call that is implemented by generic tree actions

class ToolCallDeclareAction : public BT::StatefulActionNode
{
public:
	  ToolCallDeclareAction(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {
        node_ = ROSCommon::GetInstance()->GetNode();        
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("tc_definition")};
    }

    BT::NodeStatus onStart() {
        std::string tc_definition;
        if (!getInput<std::string>("tc_definition", tc_definition)) {
      		throw BT::RuntimeError("missing tc_definition");
        }

        if (tc_definition.size() < 3) {
      		throw BT::RuntimeError("invalid tc_definition");
        }

        // Since the tc def needs the opening and closing braces to be escaped (since braces mean
        // something special to the tree parser), remove the escapes.
        tc_definition = std::regex_replace(tc_definition, std::regex(R"(\\\{)"), "{");
        tc_definition = std::regex_replace(tc_definition, std::regex(R"(\\\})"), "}");

        ToolCallData& tc_data = ToolCallData::getInstance();
        tc_data.declare_tool(tc_definition);
        RCLCPP_INFO(node_->get_logger(), "Tool call declared: tool definition: %s", tc_definition.c_str());
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
