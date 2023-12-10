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

#include <iostream>
#include <fstream>
#include <string>
#include <regex>

#include <algorithm>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <behaviortree_cpp_v3/action_node.h>

class TextExtractAction : public BT::SyncActionNode
{
    public:
		TextExtractAction(const std::string& name, const BT::NodeConfiguration& config)
				: BT::SyncActionNode(name, config)
		{
			setlocale(LC_ALL, "");
		}

		static BT::PortsList providedPorts()
		{
			return{ BT::InputPort<std::string>("pattern"),
				    BT::InputPort<std::string>("text"),
					BT::OutputPort<std::string>("field1"),
					BT::OutputPort<std::string>("field2")};
		}

		virtual BT::NodeStatus tick() override
		{
			std::string pattern;
			getInput<std::string>("pattern", pattern);

			std::string text;
			getInput<std::string>("text", text);

			ToLower(text);

			if (pattern.length() == 0) {
				if (text.length() == 0) {
					return BT::NodeStatus::SUCCESS;
				} else {
					return BT::NodeStatus::FAILURE;
				}
			}

			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pattern: [%s], text: [%s]", pattern.c_str(), text.c_str());

			std::smatch m;
			if (std::regex_match(text, m, std::regex(pattern))) {
				setOutput("field1", m[1].str());
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Match1 [%s]", m[1].str().c_str());
				if (m.size() >= 3) {
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Match2 [%s]", m[2].str().c_str());
					setOutput("field2", m[2].str());
				}
				return BT::NodeStatus::SUCCESS;
			} else {
				return BT::NodeStatus::FAILURE;
			}
		}

		void ToLower(std::string &str)
		{
			std::transform(str.begin(), str.end(), str.begin(),
					[](unsigned char c) { return std::tolower(c); });
		}

    private:
};

