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

/*
ToolRunnerAction

Runs tool calls by creating a subtree on the fly that calls the actual subtree
(defined elsewhere) that implements the tool call.

Inputs:
JSON string describing the tool call(s)

Outputs:
JSON string describing the results of each tool call
*/

#pragma once

#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include <behaviortree_cpp/action_node.h>

namespace BT
{
class Factory;
class Tree;
}

class ToolCallRunnerAction : public BT::StatefulActionNode
{
public:
	ToolCallRunnerAction(const std::string& name, const BT::NodeConfig& config,
						 std::shared_ptr<BT::BehaviorTreeFactory> factory,
						 std::shared_ptr<BT::Blackboard> global_bb) :
		BT::StatefulActionNode(name, config),
		factory_(factory),
		global_bb_(global_bb)
    {}

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<bool>("run_tools"),
			BT::InputPort<std::string>("parallel_tool_mode"),
			BT::BidirectionalPort<std::string>("tool_calls"),
			BT::OutputPort<std::string>("tool_call_results")
		};
	}

	BT::NodeStatus onStart();
    BT::NodeStatus onRunning();
    void onHalted();

private:
	void handle_start_failure_cleanup(const nlohmann::json &tc_array);

	std::shared_ptr<BT::BehaviorTreeFactory> factory_;
	std::shared_ptr<BT::Blackboard> global_bb_;

	std::string parallel_tool_mode_;								// The mode to use when running parallel tool calls
	std::map<std::string, std::string> tool_id_to_tool_name_;		// Maps tool id to tool name
	std::map<std::string, std::string> results_;					// Maps tool call id to tool result
																	// Maps tool id to a pair holding the subtree and blackboard
																	// used for running the tool
	std::map<std::string, std::pair<BT::Tree, std::shared_ptr<BT::Blackboard>>> running_trees_;
};
