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
#include "std_msgs/msg/bool.hpp"
#include <behaviortree_cpp/action_node.h>
#include "ros_common.hpp"

// Singleton for subscribing to object location message - shared by all VoiceDetectedActionAction node instances
class VoiceDetectedActionROSNodeIf
{
public:
	VoiceDetectedActionROSNodeIf(VoiceDetectedActionROSNodeIf const&) = delete;
	VoiceDetectedActionROSNodeIf& operator=(VoiceDetectedActionROSNodeIf const&) = delete;

    static std::shared_ptr<VoiceDetectedActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<VoiceDetectedActionROSNodeIf> s{new VoiceDetectedActionROSNodeIf(node)};
        return s;
    }

    void update()
    {
        //ROSCommon::GetInstance()->spin_some();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vad_sub_;    
    bool vad_{false};

private:
    VoiceDetectedActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
        vad_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "speech_detect/vad",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&VoiceDetectedActionROSNodeIf::vadCallback, this, std::placeholders::_1));
    }

    void vadCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        vad_ = msg->data;
    }
};

class VoiceDetectedAction : public BT::SyncActionNode
{
    public:
		VoiceDetectedAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
            node_if_ = VoiceDetectedActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return{};
        }

        virtual BT::NodeStatus tick() override
        {
            node_if_->update();
            if (node_if_->vad_) {
				RCLCPP_INFO(node_if_->node_->get_logger(), "Voice Activity Detected");
            	return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<VoiceDetectedActionROSNodeIf> node_if_;
};
