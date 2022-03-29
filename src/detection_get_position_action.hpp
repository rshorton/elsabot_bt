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

/*
DetectionGetPositionAction

Get the position of the specified object that is currently in the
collection of detected objects.

Inputs:
det - object detector instance to use
obj_id - ID of the object
coord_frame - coordinate frame in which to transform the object position

Outputs:
pose - Position of the object as string, format: "<x>,<y>,<yaw>"
*/

#pragma once

#include <stdio.h>
#include <sstream>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "detection_processor_container.hpp"
#include "robot_status.hpp"
#include "ros_common.hpp"

class DetectionGetPositionAction : public BT::SyncActionNode
{
    public:
	DetectionGetPositionAction(const std::string& name, const BT::NodeConfiguration& config) :
		BT::SyncActionNode(name, config)
    {
    }

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("det"),
			BT::InputPort<size_t>("obj_id"),
			BT::InputPort<std::string>("coord_frame"),
			BT::OutputPort<std::string>("position")
		};
	}

	virtual BT::NodeStatus tick() override
	{
        rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

		std::string det;
		if (!getInput<std::string>("det", det)) {
			throw BT::RuntimeError("missing det");
		}

		size_t obj_id;
		if (!getInput<size_t>("obj_id", obj_id)) {
			throw BT::RuntimeError("missing obj_id");
		}

		std::string coord_frame;
		if (!getInput<std::string>("coord_frame", coord_frame)) {
			throw BT::RuntimeError("missing coord_frame");
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

		double x, y, z = 0.0;
		if (!proc->GetObjectPos(obj_id, x, y, z, coord_frame)) {
			RCLCPP_INFO(node->get_logger(), "Object [%lu] does not exist", obj_id);
			return BT::NodeStatus::FAILURE;
		}

		std::stringstream ss;
		ss << x << ',' << y << ',' << z;
		setOutput("position", ss.str());
		RCLCPP_INFO(node->get_logger(), "Object [%lu] pos [%s]", obj_id, ss.str().c_str());
		return BT::NodeStatus::SUCCESS;
	}

};
