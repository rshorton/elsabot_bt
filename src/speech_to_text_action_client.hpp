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

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "speech_action_interfaces/action/recognize.hpp"

#include "std_msgs/msg/header.hpp"
#include "behaviortree_cpp/action_node.h"

class SpeechToTextActionClient : public BT::SyncActionNode
{
public:
	using Recognize = speech_action_interfaces::action::Recognize;
	using GoalHandleRecognize = rclcpp_action::ClientGoalHandle<Recognize>;

    SpeechToTextActionClient(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config)
    {
    	node_ = node;
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<std::string>("text") };
    }

    virtual BT::NodeStatus tick() override
    {
    	std::stringstream node_name;
    	{
    		const std::lock_guard<std::mutex> lock(_mutex);
    		node_name << "speech_to_text_action_client" << instance++;
    	}
        //node_ = rclcpp::Node::make_shared(node_name.str());

        auto action_client = rclcpp_action::create_client<speech_action_interfaces::action::Recognize>(node_, "recognize");
        // if no server is present, fail after 10 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server Speech Recognize not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        _aborted = false;

        RCLCPP_INFO(node_->get_logger(), "Sending speech-to-text goal");

        auto goal_msg = Recognize::Goal();
        goal_msg.timeout = 10;

        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<speech_action_interfaces::action::Recognize>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(node_, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<speech_action_interfaces::action::Recognize>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
            	std::string text = wrapped_result.result->text;
				RCLCPP_INFO(node_->get_logger(), "Recognized [%s]", text.c_str());
				if (!text.compare("ERROR")) {
					text = "";
				}
				setOutput("text", text);
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                return BT::NodeStatus::FAILURE;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                return BT::NodeStatus::FAILURE;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                return BT::NodeStatus::FAILURE;
        }

        if (_aborted) {
            // this happens only if method halt() was invoked
            //_client.cancelAllGoals();
            RCLCPP_INFO(node_->get_logger(), "Speech to text aborted");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "result received");
        return BT::NodeStatus::SUCCESS;
    }
#if 0
    virtual void halt() override {
        _aborted = true;
    }

#endif
private:
    bool _aborted;
    rclcpp::Node::SharedPtr node_;
    std::mutex _mutex;
    static int instance;
};

int SpeechToTextActionClient::instance = 0;

