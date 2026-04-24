/*
Copyright 2026 Scott Horton

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

#include <stdio.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <behaviortree_cpp/action_node.h>

// Singleton for subscribing to audio output TTS status
class TTSActiveActionROSNodeIf
{
public:
	TTSActiveActionROSNodeIf(TTSActiveActionROSNodeIf const&) = delete;
	TTSActiveActionROSNodeIf& operator=(TTSActiveActionROSNodeIf const&) = delete;

    static std::shared_ptr<TTSActiveActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<TTSActiveActionROSNodeIf> s{new TTSActiveActionROSNodeIf(node)};
        return s;
    }

    void update()
    {
    	rclcpp::spin_some(node_);
    }

    rclcpp::Node::SharedPtr node_;
    bool tts_active_;

private:
    TTSActiveActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
    	tts_active_(false)
	{
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/audio_output/status/tts",
			rclcpp::SystemDefaultsQoS(),
			std::bind(&TTSActiveActionROSNodeIf::callback, this, std::placeholders::_1));
    }

    void callback(std_msgs::msg::String::SharedPtr msg)
    {
    	tts_active_ = msg->data == "active";
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

class TTSActiveAction : public BT::SyncActionNode
{
    public:
	TTSActiveAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = TTSActiveActionROSNodeIf::instance(node);
        }

        virtual BT::NodeStatus tick() override
        {
           	node_if_->update();
            if (node_if_->tts_active_) {
                return BT::NodeStatus::SUCCESS; 
            }
    		return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<TTSActiveActionROSNodeIf> node_if_;
};
