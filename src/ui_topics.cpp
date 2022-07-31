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
This class is used to create and contain Object Detection Processor
instances.  A tree node can request the creation and other nodes can
query the status of the detector at any time. 
*/

#include <iostream>
#include <string>
#include <map>
#include <chrono>

#include "ui_topics.hpp"

UITopics *UITopics::ui_topics_ = nullptr;

UITopics::UITopics(rclcpp::Node::SharedPtr node):
	node_(node)
{
    generic_sub_ = node_->create_subscription<robot_ui_interfaces::msg::Generic>(
		"/ui/generic",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&UITopics::GenericCallback, this, std::placeholders::_1));
}

void UITopics::GenericCallback(robot_ui_interfaces::msg::Generic::SharedPtr msg)
{
	RCLCPP_DEBUG(node_->get_logger(), "Generic UI msg: name: [%s], type: [%s], value: [%s], instance: [%u]",
		msg->name.c_str(), msg->type.c_str(), msg->value.c_str(), msg->instance);

	auto it = generic_msgs_.find(msg->name);
	if (it != generic_msgs_.end()) {
		if (it->second.msg.instance != msg->instance) {
			it->second.msg = *msg;
			it->second.updated = true;
			RCLCPP_DEBUG(node_->get_logger(), "Generic UI msg: name: [%s] updated", msg->name.c_str());
		}
		return;
	}
	// First time received
	GenericMsg m;
	m.msg = *msg;
	m.updated = true;
	generic_msgs_[msg->name] = m;
	RCLCPP_INFO(node_->get_logger(), "Generic UI msg: name: [%s] added", msg->name.c_str());
}

bool UITopics::GetGeneric(const std::string &name, std::string &type, std::string &value, bool &updated)
{
	auto it = generic_msgs_.find(name);
	if (it != generic_msgs_.end()) {
		type = it->second.msg.type;
		value = it->second.msg.value;
		updated = it->second.updated;
		it->second.updated = false;
		return true;
	}
	return false;
}

void UITopics::SetGeneric(const std::string &name, const std::string &value)
{
	auto it = generic_msgs_.find(name);
	if (it != generic_msgs_.end()) {
		it->second.msg.value = value;
		it->second.updated = false;
	} else {
		GenericMsg m;
		m.msg.type = "";
		m.msg.value = value;
		m.updated = false;
		generic_msgs_[name] = m;
	}
}
