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

#include "tool_call_data.hpp"

// Use this action to specify the subtree used to run an AI tool.

class ToolCallSetRunnerSubtreeAction : public BT::StatefulActionNode
{
public:
	ToolCallSetRunnerSubtreeAction(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("tool_name"),
                 BT::InputPort<std::string>("subtree_name")
               };
    }

    BT::NodeStatus onStart() {
        std::string tool_name;
        if (!getInput<std::string>("tool_name", tool_name)) {
      		throw BT::RuntimeError("missing tool_name");
        }
        std::string subtree_name;
        if (!getInput<std::string>("subtree_name", subtree_name)) {
      		throw BT::RuntimeError("missing subtree_name");
        }

        ToolCallData& tc_data = ToolCallData::getInstance();
        tc_data.set_subtree_runner(tool_name, subtree_name);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() {
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() {
    }

private:
};
