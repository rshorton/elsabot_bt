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

#include <ctime>

#include <behaviortree_cpp/action_node.h>

class TimeSinceAction : public BT::SyncActionNode
{
    public:
		TimeSinceAction(const std::string& name, const BT::NodeConfig& config)
				: BT::SyncActionNode(name, config)
		{
		}

		static BT::PortsList providedPorts()
		{
			return{ BT::InputPort<std::chrono::time_point<std::chrono::steady_clock>>("time_point"),
				    BT::InputPort<std::string> ("comparison"),
					BT::InputPort<float> ("duration_sec") };
		}

		virtual BT::NodeStatus tick() override
		{
			std::chrono::time_point<std::chrono::steady_clock> time_point;
			if (!getInput<std::chrono::time_point<std::chrono::steady_clock>>("time_point", time_point)) {
				throw BT::RuntimeError("missing time_point");
			}

			std::string comparison = "greater";
			getInput<std::string>("comparison", comparison);

			float duration_sec = 0.0f;
			if (!getInput<float>("duration_sec", duration_sec)) {
				throw BT::RuntimeError("missing duration_sec");
			}

			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(now - time_point).count();

			if (comparison == "greater") {
				if (elapsed >= duration_sec) {
					return BT::NodeStatus::SUCCESS; 
				}
			} else if (comparison == "less") {
				if (elapsed < duration_sec) {
					return BT::NodeStatus::SUCCESS; 
				}
			}
			return BT::NodeStatus::FAILURE;
		}

    private:
};

