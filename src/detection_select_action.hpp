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
#include <behaviortree_cpp_v3/action_node.h>
#include "detection_processor_container.hpp"
#include "ros_common.hpp"

class DetectionSelectAction : public BT::SyncActionNode
{
    public:
	DetectionSelectAction(const std::string& name, const BT::NodeConfiguration& config) :
		BT::SyncActionNode(name, config)
    {
    }

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("det"),
			BT::InputPort<std::string>("obj_class"),
			BT::InputPort<std::string>("token"),
			BT::OutputPort<size_t>("obj_id"),
			BT::OutputPort<std::string>("count"),
		};
	}

	virtual BT::NodeStatus tick() override
	{
		rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

		std::string det;
		if (!getInput<std::string>("det", det)) {
			throw BT::RuntimeError("missing det");
		}

		std::string obj_class;
		if (!getInput<std::string>("obj_class", obj_class)) {
			throw BT::RuntimeError("missing obj_class");
		}

		std::string token;
		if (!getInput<std::string>("token", token)) {
			throw BT::RuntimeError("missing token");
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

		int count = 0;
		proc->DeSelect();
		if (proc->Select(obj_class, ObjDetProc::SelectionMetric::closest, token, count)) {
			ObjDetProc::Detection det;
			if (proc->GetSelected(det)) {
				RCLCPP_INFO(node->get_logger(), "SelectObjectAction: GetSelected returned: id: [%lu], dist: [%f], x,y,z,yaw: %f, %f, %f, %f, token: [%s]",
					det.id, det.dist, det.pos.x, det.pos.y, det.pos.z, det.yaw, det.token.c_str());	
				setOutput("obj_id", det.id);
				std::stringstream ss;
				ss << count;
				setOutput("count", ss.str());
				return BT::NodeStatus::SUCCESS;
			} else {
				RCLCPP_INFO(node->get_logger(), "SelectObjectAction: GetSelected failed");	
			}
		} else {
			RCLCPP_INFO(node->get_logger(), "TestSelect: Select failed");
		}			
		return BT::NodeStatus::FAILURE;
	}
};
