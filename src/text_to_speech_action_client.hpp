#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "speech_action_interfaces/action/speak.hpp"

#include "std_msgs/msg/header.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class TextToSpeechActionClient : public BT::SyncActionNode
{
public:
	using Speak = speech_action_interfaces::action::Speak;
	using GoalHandleSpeak = rclcpp_action::ClientGoalHandle<Speak>;

    TextToSpeechActionClient(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        ///node_ = rclcpp::Node::make_shared("text_to_speech_action_client");
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("msg"), BT::InputPort<std::string>("msg2") };
    }

    void ReplaceAllOccurrences(std::string search, std::string replace, std::string &str)
    {
    	size_t pos = 0;
    	size_t len = str.length();
    	size_t slen = search.length();
    	size_t rlen = replace.length();
    	while (pos < len) {
    		size_t found = str.find(search, pos);
    		if (found != std::string::npos) {
	        	str = str.replace(found, slen, replace);
	        	len = str.length();
	        	pos = found + rlen + 1;
    		} else {
    			break;
    		}
    	}
    }

    void UnescapeXML(std::string &str)
    {
    	ReplaceAllOccurrences("$lt;",   "<", str);
    	ReplaceAllOccurrences("$gt;",   ">", str);
    	ReplaceAllOccurrences("$quot;", "\"", str);
    	ReplaceAllOccurrences("$amp;",  "&", str);
    }

    virtual BT::NodeStatus tick() override
    {
    	std::string msg;
    	std::string msg2;
    	if (!getInput<std::string>("msg", msg)) {
			throw BT::RuntimeError("missing msg to say");
		}
    	// optional
    	getInput<std::string>("msg2", msg2);

    	//RCLCPP_INFO(node_->get_logger(), "msg [%s], msg2[%s]", msg.c_str(), msg2.c_str());

    	std::stringstream node_name;
    	{
    		const std::lock_guard<std::mutex> lock(_mutex);
    		node_name << "text_to_speech_action_client" << instance++;
    	}

        node_ = rclcpp::Node::make_shared(node_name.str());
        auto action_client = rclcpp_action::create_client<speech_action_interfaces::action::Speak>(node_, "speak");
        // if no server is present, fail after 10 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            // RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        _aborted = false;

    	msg += " ";
    	msg += msg2;

        // Unescape any SSML in the string(s)
        UnescapeXML(msg);

		std::cout << "Unescaped text: " << msg << std::endl;

    	std::ifstream file("./ssml.xml");
    	std::string ssml_in, str;
    	while (std::getline(file, str))
    	{
    		ssml_in += str;
    		ssml_in.push_back('\n');
    	}

    	size_t pos = 0;
    	std::string ssml;
    	std::string place_holder = "TEXT";
    	pos = ssml_in.find(place_holder);
    	if (pos != std::string::npos) {
        	ssml = ssml_in.replace(pos, place_holder.length(), msg);
    	}
        RCLCPP_INFO(node_->get_logger(), "Sending text-to-speech cmd [%s]", msg.c_str());
//        RCLCPP_INFO(node_->get_logger(), "Sending text-to-speech ssml cmd [%s]", ssml.c_str());

        auto goal_msg = Speak::Goal();
        goal_msg.text = ssml;
        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<speech_action_interfaces::action::Speak>::SharedPtr goal_handle = goal_handle_future.get();
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

        rclcpp_action::ClientGoalHandle<speech_action_interfaces::action::Speak>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
				RCLCPP_INFO(node_->get_logger(), "Result [%s]", wrapped_result.result->result.c_str());
                break;
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

int TextToSpeechActionClient::instance = 0;


