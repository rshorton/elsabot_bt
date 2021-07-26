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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

using namespace std;

class GetRandomSelectionAction : public BT::SyncActionNode
{
    public:
		GetRandomSelectionAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("options"), BT::OutputPort<std::string>("selected")};
        }

        virtual BT::NodeStatus tick() override
        {
    		string options;

			if (!getInput<std::string>("options", options)) {
				throw BT::RuntimeError("missing expected options");
			}

			string selected;
    		auto parts = BT::splitString(options, '|');
    		int count = parts.size();
    		if (count > 0) {
    			int idx = rand() % count;
    			selected = BT::convertFromString<std::string>(parts[idx]);
    		}
			cout << "Selected: " << selected << ", From: [" << options << "]" << endl;
			setOutput("selected", selected);
            return BT::NodeStatus::SUCCESS;
        }

    private:
};
