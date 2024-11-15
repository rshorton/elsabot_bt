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

#include "rclcpp/rclcpp.hpp"
#include "robot_head_interfaces/msg/antenna.hpp"
#include <behaviortree_cpp/action_node.h>

// Single for publishing the antenna control state - shared by all AntennaActionNode instances
class AntennaActionROSNodeIf
{
public:
	AntennaActionROSNodeIf(AntennaActionROSNodeIf const&) = delete;
	AntennaActionROSNodeIf& operator=(AntennaActionROSNodeIf const&) = delete;

    static std::shared_ptr<AntennaActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<AntennaActionROSNodeIf> s{new AntennaActionROSNodeIf(node)};
        return s;
    }

    void setAntennaState(uint32_t rate, uint32_t intensity, std::string left_blink_pattern,
    					 std::string right_blink_pattern)
    {
    	RCLCPP_INFO(node_->get_logger(), "Set Antenna: rate= %d, intensity= %d, left pattern= %s, right pattern= %s",
    				rate, intensity, left_blink_pattern.c_str(), right_blink_pattern.c_str());

    	auto message = robot_head_interfaces::msg::Antenna();
        message.rate = rate;
        message.intensity = intensity;
        message.left_blink_pattern = left_blink_pattern;
        message.right_blink_pattern = right_blink_pattern;
        antenna_publisher_->publish(message);
    }

private:
    AntennaActionROSNodeIf(rclcpp::Node::SharedPtr node):
		node_(node)
	{
        antenna_publisher_ = node_->create_publisher<robot_head_interfaces::msg::Antenna>("/head/antenna", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::Antenna>::SharedPtr antenna_publisher_;
};

class AntennaAction : public BT::SyncActionNode
{
    public:
		AntennaAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = AntennaActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("left_blink_pattern"), BT::InputPort<std::string>("right_blink_pattern"),
        		     BT::InputPort<uint32_t>("rate"), BT::InputPort<uint32_t>("intensity")};
        }

        virtual BT::NodeStatus tick() override
        {
        	uint32_t rate;
        	uint32_t intensity;
        	std::string left_blink_pattern;
        	std::string right_blink_pattern;

        	if (!getInput<uint32_t>("rate", rate)) {
    			throw BT::RuntimeError("missing rate");
    		}
        	if (!getInput<uint32_t>("intensity", intensity)) {
    			throw BT::RuntimeError("missing intensity");
    		}
        	if (!getInput<std::string>("left_blink_pattern", left_blink_pattern)) {
    			throw BT::RuntimeError("missing left_blink_pattern");
    		}
        	if (!getInput<std::string>("right_blink_pattern", right_blink_pattern)) {
    			throw BT::RuntimeError("missing right_blink_pattern");
    		}

        	node_if_->setAntennaState(rate, intensity, left_blink_pattern, right_blink_pattern);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<AntennaActionROSNodeIf> node_if_;
};
