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

class GetTimeNowAction : public BT::SyncActionNode
{
    public:
		GetTimeNowAction(const std::string& name, const BT::NodeConfig& config)
				: BT::SyncActionNode(name, config)
		{
		}

		static BT::PortsList providedPorts()
		{
			return{ BT::OutputPort<std::chrono::time_point<std::chrono::steady_clock>>("now") };
		}

		virtual BT::NodeStatus tick() override
		{
			auto now = std::chrono::steady_clock::now();
			setOutput("now", now);
			return BT::NodeStatus::SUCCESS;
		}

    private:
};

