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
