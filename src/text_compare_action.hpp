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
			node_ = rclcpp::Node::make_shared("text_compare_action");
			setlocale(LC_ALL, "");
		}

		static BT::PortsList providedPorts()
		{
			return{ BT::InputPort<std::string>("ck_for"), BT::InputPort<std::string>("text")  };
		}

		virtual BT::NodeStatus tick() override
		{
			std::string ckFor;
			if (!getInput<std::string>("ck_for", ckFor)) {
				throw BT::RuntimeError("missing text to check for");
			}

			std::string text;
			if (!getInput<std::string>("text", text)) {
				throw BT::RuntimeError("missing text to be checked");
			}

			ToLower(ckFor);
			ToLower(text);

			RCLCPP_INFO(node_->get_logger(), "Checking [%s] for [%s]", text.c_str(), ckFor.c_str());

			if (ckFor.length() == 0) {
				if (text.length() == 0) {
					return BT::NodeStatus::SUCCESS;
				} else {
					return BT::NodeStatus::FAILURE;
				}
			} else if (std::regex_match (text, std::regex(ckFor) )) {
//			} else if (text.find(ckFor) !=std::string::npos) {
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
		rclcpp::Node::SharedPtr node_;
};

