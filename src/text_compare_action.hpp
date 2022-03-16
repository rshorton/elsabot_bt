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

class TextCompareAction : public BT::SyncActionNode
{
    public:
		TextCompareAction(const std::string& name, const BT::NodeConfiguration& config)
				: BT::SyncActionNode(name, config)
		{
			setlocale(LC_ALL, "");
		}

		static BT::PortsList providedPorts()
		{
			return{ BT::InputPort<std::string>("ck_for"), BT::InputPort<std::string>("text")  };
		}

		virtual BT::NodeStatus tick() override
		{
			std::string ckFor;
			getInput<std::string>("ck_for", ckFor);

			std::string text;
			getInput<std::string>("text", text);

			ToLower(ckFor);
			ToLower(text);

			//RCLCPP_INFO(node_->get_logger(), "Checking [%s] for [%s]", text.c_str(), ckFor.c_str());

			if (ckFor.length() == 0) {
				if (text.length() == 0) {
					return BT::NodeStatus::SUCCESS;
				} else {
					return BT::NodeStatus::FAILURE;
				}
			} else if (std::regex_match (text, std::regex(ckFor) )) {
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

