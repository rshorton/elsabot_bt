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

#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "detection_processor_container.hpp"
#include "ros_common.hpp"
#include "tool_call_data.hpp"
#include "tool_call_runner_action.hpp"

using json = nlohmann::json;

BT::NodeStatus ToolCallRunnerAction::onStart() {
	auto node = ROSCommon::GetInstance()->GetNode();

	running_trees_.clear();
	results_.clear();

	bool run_tools = false;
	if (!getInput<bool>("run_tools", run_tools)) {
		throw BT::RuntimeError("missing run_tools");
	}

	if (!run_tools) {
		return BT::NodeStatus::SUCCESS;	
	}

	std::string toolcalls;
	if (!getInput<std::string>("tool_calls", toolcalls)) {
		throw BT::RuntimeError("missing tool_calls");
	}

	if (toolcalls.empty()) {
		RCLCPP_DEBUG(node->get_logger(), "%s, No tools specified", name().c_str());
		return BT::NodeStatus::SUCCESS;	
	}

	setOutput("tool_calls", "");
	setOutput("tool_call_results", "");

	if (!getInput<std::string>("parallel_tool_mode", parallel_tool_mode_)) {
		throw BT::RuntimeError("missing parallel_tool_mode");
	}

	json tc_array;
  	try {
    	tc_array = json::parse(toolcalls);
  	} catch (json::parse_error& ex) {
    	RCLCPP_ERROR(node->get_logger(), "%s, Failed to parse toolcalls, msg: %s, at: %ld", name().c_str(),
					 toolcalls.c_str(), ex.byte);
    	return BT::NodeStatus::FAILURE;
  	}

	ToolCallData& tc_data = ToolCallData::getInstance();

	// Create a subtree xml for each tool call.  This subtree will call the actual subtree that implements the tool.
	// A typical run of this node uses only one subtree.  Multiple trees are used when parallel calls are made.
	size_t idx = 0;
	for (const auto &tc: tc_array) {
		std::string tool_call_id = tc["id"];
		std::string tool_name = tc["function"]["name"];
    	std::string tool_call_args_json = tc["function"]["arguments"].dump();

		std::string subtree_name;
		if (!tc_data.get_subtree_runner(tool_name, subtree_name)) {
			RCLCPP_ERROR(node->get_logger(), "%s, Cannot execute tool call, missing subtree runner, tool: %s", name().c_str(), tool_name.c_str());
			handle_start_failure_cleanup(tc_array);
			return BT::NodeStatus::FAILURE;
		}

		auto subtree_id = "ToolCall" + std::to_string(idx);

		std::stringstream ss;
		ss << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"" << subtree_id << "\"><Sequence><SubTree ID=\""
		   << subtree_name << "\" _autoremap=\"true\" /></Sequence></BehaviorTree></root>";

		factory_->registerBehaviorTreeFromText(ss.str());

		// Create a blackboard for this subtree and set the tool call args.
		// The global blackboard is connected to the subtree BB so the subtree can
		// pass data to/from other nodes in the main tree (like the TTS buffer).
		auto bb = BT::Blackboard::create(global_bb_);
		bb->set("tool_call_args", tool_call_args_json);

		running_trees_.emplace(tool_call_id, std::make_pair(std::move(factory_->createTree(subtree_id, bb)), bb));
		tool_id_to_tool_name_.emplace(tool_call_id, tool_name);

		RCLCPP_INFO(node->get_logger(), "%s, Created tool subtree runner, tool: %s, id: %s, subtree: %s",
					name().c_str(), tool_name.c_str(), tool_call_id.c_str(), subtree_id.c_str());
		++idx;
	}
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ToolCallRunnerAction::onRunning() {
	auto node = ROSCommon::GetInstance()->GetNode();

	bool run_tools = false;
	getInput<bool>("run_tools", run_tools);

	// Cancel running trees if run disabled
	if (!run_tools) {
		return BT::NodeStatus::SUCCESS;
	}

	if (!getInput<std::string>("parallel_tool_mode", parallel_tool_mode_)) {
		throw BT::RuntimeError("missing parallel_tool_mode");
	}

	// Tick each of the active trees.  Save results when finished
	bool tree_done = false;
	for (auto it = running_trees_.begin(); it != running_trees_.end(); ) {
		RCLCPP_DEBUG(node->get_logger(), "%s, ticking tool runner for tool ID: %s", name().c_str(), it->first.c_str());
		auto status = it->second.first.tickOnce();
		if (status != BT::NodeStatus::RUNNING) {
			// Get the results from the subtree-specific blackboard
			auto result = it->second.second->get<std::string>("tool_call_result");
			RCLCPP_INFO(node->get_logger(), "%s, tool runner finished, tool: %s, result: %s", name().c_str(),
						tool_id_to_tool_name_[it->first].c_str(), result.c_str());

			results_.emplace(it->first, result);
			it = running_trees_.erase(it);
			tree_done = true;
		} else {
			++it;
		}
	}

	// If parallel tools are being run, and if the mode specifies stopping after
	// the first tree completes, then stop other subtrees and set result
	if (running_trees_.size() > 0 && tree_done) {
		RCLCPP_INFO(node->get_logger(), "%s, waiting for %ld tools to finish, parallel_tool_mode: %s",
					name().c_str(), running_trees_.size(), parallel_tool_mode_.c_str());

		if (parallel_tool_mode_ == "wait_one") {
			json result = {{"result", "preempted"}};
			auto result_str = result.dump();

			for (auto it = running_trees_.cbegin(); it != running_trees_.cend(); ) {
				results_.emplace(it->first, result_str);
				it = running_trees_.erase(it);
			}
		}
	}

	// If no more active trees, then report the tool results
	if (running_trees_.empty()) {
		auto results = json::array();
		for (const auto &r: results_) {
			results.push_back({{"tool_call_id", r.first}, {"tool_call_name", tool_id_to_tool_name_[r.first]}, {"result", r.second}});
		}

		auto json = results.dump();
		setOutput("tool_call_results", json);
		results_.clear();

		RCLCPP_INFO(node->get_logger(), "%s, Finished, results: %s",
					name().c_str(), json.c_str());
		return BT::NodeStatus::SUCCESS;
	}
	return BT::NodeStatus::RUNNING;
}       

void ToolCallRunnerAction::onHalted() {
	running_trees_.clear();
}

void ToolCallRunnerAction::handle_start_failure_cleanup(const json &tc_array) {
	json result = {{"result", "failured_to_start_tools"}};
	auto result_str = result.dump();

	auto results = json::array();

	for (const auto &tc: tc_array) {
		std::string tool_call_id = tc["id"];
		std::string tool_name = tc["function"]["name"];
		results.push_back({{"tool_call_id", tool_call_id}, {"tool_call_name", tool_name}, {"result", result_str}});
	}

	auto json = results.dump();
	setOutput("tool_call_results", json);
}
