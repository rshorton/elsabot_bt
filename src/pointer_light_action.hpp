/*
Copyright 2023 Scott Horton

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
#include "robot_head_interfaces/msg/pointer_light.hpp"
#include <behaviortree_cpp/action_node.h>

// Singleton for publishing the pointer light control state - shared by all PointerLightAction instances
class PointerLightActionROSNodeIf
{
public:
	PointerLightActionROSNodeIf(PointerLightActionROSNodeIf const&) = delete;
	PointerLightActionROSNodeIf& operator=(PointerLightActionROSNodeIf const&) = delete;

    static std::shared_ptr<PointerLightActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<PointerLightActionROSNodeIf> s{new PointerLightActionROSNodeIf(node)};
        return s;
    }

    void setPointerLightState(int32_t w_level, int32_t l_level)
    {

    	RCLCPP_INFO(node_->get_logger(), "Set pointer light: white_level= %d, laser_lever= %d", w_level, l_level);
    	auto message = robot_head_interfaces::msg::PointerLight();
        message.white_level = w_level;
        message.laser_level = l_level;
        pointer_light_publisher_->publish(message);
    }

private:
    PointerLightActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        pointer_light_publisher_ = node_->create_publisher<robot_head_interfaces::msg::PointerLight>("/head/pointer_light", 1);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::PointerLight>::SharedPtr pointer_light_publisher_;
};

class PointerLightAction : public BT::SyncActionNode
{
public:
    PointerLightAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config)
    {
        node_if_ = PointerLightActionROSNodeIf::instance(node);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("white_level"),
                 BT::InputPort<std::string>("laser_level") };
    }

    virtual BT::NodeStatus tick() override
    {
        int32_t white_level = 0;
        int32_t laser_level = 0;
        getInput<std::int32_t>("white_level", white_level);
        getInput<std::int32_t>("laser_level", laser_level);

        node_if_->setPointerLightState(white_level, laser_level);
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<PointerLightActionROSNodeIf> node_if_;
};
