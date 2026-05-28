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

#include <ratio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <behaviortree_cpp/action_node.h>
#include "speech_action_interfaces/msg/wakeword.hpp"
#include "ros_common.hpp"

using namespace std::chrono;

class WakeWordDetectedActionROSNodeIf
{
public:
	WakeWordDetectedActionROSNodeIf(WakeWordDetectedActionROSNodeIf const&) = delete;
	WakeWordDetectedActionROSNodeIf& operator=(WakeWordDetectedActionROSNodeIf const&) = delete;

    static std::shared_ptr<WakeWordDetectedActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<WakeWordDetectedActionROSNodeIf> s{new WakeWordDetectedActionROSNodeIf(node)};
        return s;
    }

    void update()
    {
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<speech_action_interfaces::msg::Wakeword>::SharedPtr ww_sub_;

    bool valid_{false};
    std::string ww_;
    rclcpp::Time ww_time_;

private:
    WakeWordDetectedActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
        ww_sub_ = node_->create_subscription<speech_action_interfaces::msg::Wakeword>(
            "speech_detect/wakeword",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&WakeWordDetectedActionROSNodeIf::wwCallback, this, std::placeholders::_1));
    }

    void wwCallback(speech_action_interfaces::msg::Wakeword::SharedPtr msg)
    {
        ww_ = msg->word;
        ww_time_ = msg->stamp;
        valid_ = true;
        RCLCPP_INFO(node_->get_logger(), "Wakeword Detected: %s", ww_.c_str());
    }
};

class WakeWordDetectedAction : public BT::SyncActionNode
{
    public:
	WakeWordDetectedAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)

        {
            node_if_ = WakeWordDetectedActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return {BT::InputPort<float>("since_sec"),
                    BT::InputPort<bool>("clear")};
        }

        virtual BT::NodeStatus tick() override
        {
            node_if_->update();
        	if (node_if_->ww_.length()) {
				float since_sec = 4.0;
				getInput("since_sec", since_sec);

				double diff = (node_if_->node_->now() - node_if_->ww_time_).seconds();

				auto ww_temp = node_if_->ww_;

                auto clear = true;
                getInput("clear", clear);

                if (clear) {
				    node_if_->ww_ = "";
                }                    
				std::cout << "Wakeword: since last: " << diff << ", since_sec: " << since_sec << std::endl;
				if (ww_temp.length() > 0 && diff < since_sec) {
					RCLCPP_INFO(node_if_->node_->get_logger(), "Wakeword Detected");
					return BT::NodeStatus::SUCCESS;
				}
        	}
            return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<WakeWordDetectedActionROSNodeIf> node_if_;
};
