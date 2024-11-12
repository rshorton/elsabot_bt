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

class DetectionConfigureAction : public BT::SyncActionNode
{
    public:
	DetectionConfigureAction(const std::string& name, const BT::NodeConfig& config) :
		BT::SyncActionNode(name, config)
    {
    }

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("det"),
			BT::InputPort<std::string>("class_conf_mapping"),
			BT::InputPort<int>("min_det_count"),
			BT::InputPort<int>("drop_det_count"),
			BT::InputPort<int>("det_timeout"),
			BT::InputPort<std::string>("pub_topic")
		};
	}

	virtual BT::NodeStatus tick() override
	{
		rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

		std::string det;
		if (!getInput<std::string>("det", det)) {
			throw BT::RuntimeError("missing det");
		}

		std::string class_conf_mapping;
		if (!getInput<std::string>("class_conf_mapping", class_conf_mapping)) {
			throw BT::RuntimeError("missing class_conf_mapping");
		}

		int min_det_count = 5;
		getInput<int>("min_det_count", min_det_count);

		int drop_det_count = 3;
		getInput<int>("drop_det_count", drop_det_count);

		int det_timeout = 1000;
		getInput<int>("det_timeout", det_timeout);

		std::string pub_topic;
		if (!getInput<std::string>("pub_topic", pub_topic)) {
			throw BT::RuntimeError("missing pub_topic");
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

		// Convert class_conf_mapping to a map
		// Format:  class conf, class conf, ...
		auto parts = BT::splitString(class_conf_mapping, ',');
		if (parts.size() == 0) {
			throw BT::RuntimeError("invalid class_conf_mapping (empty)");
		}

		std::map<std::string, double> objects_to_det;
		for (const auto &p: parts) {
			// Space separated class-conf pair
			auto cc = BT::splitString(p, ' ');
			if (cc.size() != 2) {
				throw BT::RuntimeError("invalid class_conf_mapping pair");
			}
			std::string obj_class = BT::convertFromString<std::string>(cc[0]);
			objects_to_det[obj_class] = BT::convertFromString<double>(cc[1]);
		}

		RCLCPP_INFO(node->get_logger(), "Configure object detection processor [%s] with class-conf-map [%s], min_det_count [%d], " \
				"drop_det_count [%d], det_timeout [%d], pub_topic [%s]",
				det.c_str(),
				class_conf_mapping.c_str(),
				min_det_count,
				drop_det_count,
				det_timeout,
				pub_topic.c_str());

		if (!proc->ObjDetProc::Configure(objects_to_det, min_det_count, drop_det_count,
										 det_timeout, pub_topic)) {
			return BT::NodeStatus::FAILURE;
		}
		return BT::NodeStatus::SUCCESS;
	}
};
