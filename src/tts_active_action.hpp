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
#include <chrono>

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

class TTSActiveAction : public BT::StatefulActionNode
{
    public:
	TTSActiveAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::StatefulActionNode(name, config)
        {
			node_if_ = TTSActiveActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts() {
            return {BT::InputPort<float>("inactive_wait_timeout_sec"),  // Wait for TTS to be active up to this long
                    BT::InputPort<BT::NodeStatus>("return_on_active"),  // Node status to return if active
                    BT::OutputPort<bool>("result")};                    // Actual TTS state when node finished
        }

        BT::NodeStatus onStart() {
            inactive_wait_timeout_sec_ = 0.0f;
            getInput<float>("inactive_wait_timeout_sec", inactive_wait_timeout_sec_);

            return_on_active_ = BT::NodeStatus::SUCCESS;
            getInput<BT::NodeStatus>("return_on_active", return_on_active_);

            start_time_ = std::chrono::steady_clock::now();
            return onRunning();
        }

        BT::NodeStatus onRunning() {
           	node_if_->update();
            auto is_active = node_if_->tts_active_;

            // If no wait, just return current state
            if (inactive_wait_timeout_sec_ == 0.0f) {
                setOutput("result", is_active);
                return get_return(is_active);

            // If wait, then return after timeout or when TTS becomes inactive
            } else {
                if (!is_active) {
                    setOutput("result", false);
                    return get_return(is_active);
                }

                auto now = std::chrono::steady_clock::now();
	            auto elapsed_sec = std::chrono::duration_cast<std::chrono::duration<float>>(now - start_time_).count();
                if (elapsed_sec > inactive_wait_timeout_sec_) {
                    setOutput("result", is_active);
                    return get_return(is_active);
                }
            }
            return BT::NodeStatus::RUNNING;    
        }            

        void onHalted() {
        }

    private:
        BT::NodeStatus get_return(bool is_active) {
            if (is_active) {
                return return_on_active_;
            } else if (return_on_active_ == BT::NodeStatus::SUCCESS) {
                return BT::NodeStatus::FAILURE;
            } else {
                return BT::NodeStatus::SUCCESS; 
            }                    
        }

        std::shared_ptr<TTSActiveActionROSNodeIf> node_if_;
        float inactive_wait_timeout_sec_{0.0f};
        BT::NodeStatus return_on_active_{BT::NodeStatus::SUCCESS};

        std::chrono::time_point<std::chrono::steady_clock> start_time_;
};
