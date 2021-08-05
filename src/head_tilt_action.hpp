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
#include "robot_head_interfaces/msg/head_tilt.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Singleton for publishing the HeadTilt control state - shared by all HeadTiltActionNode instances
class HeadTiltActionROSNodeIf
{
public:
	HeadTiltActionROSNodeIf(HeadTiltActionROSNodeIf const&) = delete;
	HeadTiltActionROSNodeIf& operator=(HeadTiltActionROSNodeIf const&) = delete;

    static std::shared_ptr<HeadTiltActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
        static std::shared_ptr<HeadTiltActionROSNodeIf> s{new HeadTiltActionROSNodeIf(node)};
        return s;
    }

    void setHeadTiltState(int32_t angle, int32_t dwell_ms)
    {
    	RCLCPP_INFO(node_->get_logger(), "Set head tilt: angle= %d, dwell_ms= %d", angle, dwell_ms);
    	auto message = robot_head_interfaces::msg::HeadTilt();
        message.angle = angle;
        message.dwell_duration = dwell_ms;
        message.transition_duration = 0;
        head_tilt_publisher_->publish(message);
    }

private:
    HeadTiltActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        head_tilt_publisher_ = node_->create_publisher<robot_head_interfaces::msg::HeadTilt>("/head/tilt", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::HeadTilt>::SharedPtr head_tilt_publisher_;
};

class HeadTiltAction : public BT::SyncActionNode
{
    public:
		HeadTiltAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = HeadTiltActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("angle"), BT::InputPort<std::string>("dwell_ms") };
        }

        virtual BT::NodeStatus tick() override
        {
        	int32_t angle = 2;
        	int32_t dwell_ms = 2000;
        	if (!getInput<std::int32_t>("angle", angle)) {
    			throw BT::RuntimeError("missing angle");
    		}
        	if (!getInput<std::int32_t>("dwell_ms", dwell_ms)) {
    			throw BT::RuntimeError("missing dwell_ms");
    		}

        	node_if_->setHeadTiltState(angle, dwell_ms);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<HeadTiltActionROSNodeIf> node_if_;
};
