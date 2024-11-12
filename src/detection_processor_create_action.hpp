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
#include <limits>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/action_node.h>
#include "detection_processor_container.hpp"
#include "ros_common.hpp"

class DetectionProcessorCreateAction : public BT::SyncActionNode
{
    public:
	DetectionProcessorCreateAction(const std::string& name, const BT::NodeConfig& config) :
		BT::SyncActionNode(name, config)
	{
	}

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("det"),
			BT::InputPort<std::string>("topic")};
	}

	virtual BT::NodeStatus tick() override
	{
		rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

		std::string det;
		if (!getInput<std::string>("det", det)) {
			throw BT::RuntimeError("missing det");
		}

		std::string topic;
		if (!getInput<std::string>("topic", topic)) {
			throw BT::RuntimeError("missing topic");
		}

		RCLCPP_INFO(node->get_logger(), "Create object detection processor [%s] for topic [%s]",
				det.c_str(),
				topic.c_str());

		ObjDetProcContainer *container = ObjDetProcContainer::GetInstance();
		if (!container) {
			return BT::NodeStatus::FAILURE;
		}

		if (!container->CreateProc(det, topic)) {
			return BT::NodeStatus::FAILURE;
		}
		return BT::NodeStatus::SUCCESS;
	}
};
