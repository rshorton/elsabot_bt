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

#include <stdio.h>
#include <sstream>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/action_node.h>
#include "detection_processor_container.hpp"
#include "ros_common.hpp"

class DetectionCommandAction : public BT::SyncActionNode
{
    public:
	DetectionCommandAction(const std::string& name, const BT::NodeConfig& config) :
		BT::SyncActionNode(name, config)
    {
    }

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("det"),
			BT::InputPort<std::string>("command"),
		};
	}

	virtual BT::NodeStatus tick() override
	{
		rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

		std::string det;
		if (!getInput<std::string>("det", det)) {
			throw BT::RuntimeError("missing det");
		}

		std::string command;
		if (!getInput<std::string>("command", command)) {
			throw BT::RuntimeError("missing command");
		}

		ObjDetProcContainer *container = ObjDetProcContainer::GetInstance();
		if (!container) {
			return BT::NodeStatus::FAILURE;
		}

		std::shared_ptr<ObjDetProc> proc = container->GetProc(det);
		if (!proc) {
			RCLCPP_INFO(node->get_logger(), "Object detection processor [%s] does not exist",
				det.c_str());
			return BT::NodeStatus::FAILURE;
		}

		if (command == "clear") {
			proc->ClearCollection();
		} else if (command == "reset") {
			proc->Reset();
		} else if (command == "pause") {
			proc->Pause();
		} else if (command == "resume") {
			proc->Resume();
		}
		return BT::NodeStatus::SUCCESS;
	}
};
