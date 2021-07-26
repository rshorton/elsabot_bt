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
#include "robot_head_interfaces/msg/smile.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Single for publishing the smile control state - shared by all SmileActionNode instances
class SmileActionROSNodeIf
{
public:
	SmileActionROSNodeIf(SmileActionROSNodeIf const&) = delete;
	SmileActionROSNodeIf& operator=(SmileActionROSNodeIf const&) = delete;

    static std::shared_ptr<SmileActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<SmileActionROSNodeIf> s{new SmileActionROSNodeIf(node)};
        return s;
    }

    void setSmileState(int32_t level, int32_t duration)
    {

    	RCLCPP_INFO(node_->get_logger(), "Set smile: level= %d, duration= %d", level, duration);
    	auto message = robot_head_interfaces::msg::Smile();
        message.mode = "smile";
        message.level = level;
        message.duration_ms = duration;
        message.use_as_default = false;
        smile_publisher_->publish(message);
    }

private:
    SmileActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        smile_publisher_ = node_->create_publisher<robot_head_interfaces::msg::Smile>("/head/smile", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::Smile>::SharedPtr smile_publisher_;
};

class SmileAction : public BT::SyncActionNode
{
    public:
		SmileAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
            node_if_ = SmileActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("level"), BT::InputPort<std::string>("duration_ms") };
        }

        virtual BT::NodeStatus tick() override
        {
        	int32_t level = 2;
        	int32_t duration = 2000;
        	if (!getInput<std::int32_t>("level", level)) {
    			throw BT::RuntimeError("missing level");
    		}
        	if (!getInput<std::int32_t>("duration_ms", duration)) {
    			throw BT::RuntimeError("missing duration");
    		}

        	node_if_->setSmileState(level, duration);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<SmileActionROSNodeIf> node_if_;
};
