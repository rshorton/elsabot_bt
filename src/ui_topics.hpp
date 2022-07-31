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

#ifndef _UI_TOPICS_HPP_
#define _UI_TOPICS_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "robot_ui_interfaces/msg/generic.hpp"

class UITopics
{
private:

	struct GenericMsg {
		robot_ui_interfaces::msg::Generic msg;
		bool updated;
	};

public:
	static UITopics* Create(rclcpp::Node::SharedPtr node)
	{
		if (!ui_topics_) {
			ui_topics_ = new UITopics(node);
		}			
		return ui_topics_;
	};

	static UITopics* GetInstance()
	{
		return ui_topics_;
	};

	bool GetGeneric(const std::string &name, std::string &type, std::string &value, bool &updated);
	void SetGeneric(const std::string &name, const std::string &value);

private:
	UITopics(rclcpp::Node::SharedPtr node);
	~UITopics() {};

	void GenericCallback(robot_ui_interfaces::msg::Generic::SharedPtr msg);

private:
	static UITopics *ui_topics_;

	rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_ui_interfaces::msg::Generic>::SharedPtr generic_sub_;

	std::unordered_map<std::string, GenericMsg> generic_msgs_;
};

#endif //_UI_TOPICS_HPP_
