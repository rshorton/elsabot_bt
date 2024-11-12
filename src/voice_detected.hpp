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

class VoiceDetected : public BT::SyncActionNode
{
    public:
		VoiceDetected(const std::string& name, const BT::NodeConfig& config)
            : BT::SyncActionNode(name, config),
	        vad(false)
        {
            node_ = rclcpp::Node::make_shared("voice_activity_det_node");

            vad_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
              "speech_detect/vad",
              rclcpp::SystemDefaultsQoS(),
              std::bind(&VoiceDetected::vadCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts()
        {
        	return{};
        }

        virtual BT::NodeStatus tick() override
        {
        	rclcpp::spin_some(node_);
            if (vad) {
            	return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        void vadCallback(std_msgs::msg::Bool::SharedPtr msg)
        {
        	vad = msg->data;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vad_sub_;
        bool vad;
};
