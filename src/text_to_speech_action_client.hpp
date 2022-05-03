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

#include "game_settings.hpp"

class TextToSpeechActionClient : public BT::SyncActionNode
{
public:
	using Speak = speech_action_interfaces::action::Speak;
	using GoalHandleSpeak = rclcpp_action::ClientGoalHandle<Speak>;

    TextToSpeechActionClient(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config)
    {
    	node_ = node;

    	ssml_ = "<speak version=\"1.0\" xmlns=\"http://www.w3.org/2001/10/synthesis\" xmlns:mstts=\"https://www.w3.org/2001/mstts\" xml:lang=\"en-US\">" \
    				"<voice name=\"VOICE\">" \
						"<mstts:express-as style=\"STYLE\">" \
						"<prosody rate=\"RATE%\" pitch=\"PITCH%\" contour=\"CONTOUR\">" \
		    		        "TEXT" \
			    	    "</prosody>" \
						"</mstts:express-as>" \
					"</voice>" \
				"</speak>";
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("msg1"),
        	    BT::InputPort<unsigned long>("msg1_ul"),
        	    BT::InputPort<float>("msg1_f"),
        	    BT::InputPort<std::string>("msg2"),
        	    BT::InputPort<unsigned long>("msg2_ul"),
        	    BT::InputPort<float>("msg2_f"),
        	    BT::InputPort<std::string>("msg3"),
        	    BT::InputPort<unsigned long>("msg3_ul"),
        	    BT::InputPort<float>("msg3_f"),
        	    BT::InputPort<std::string>("msg4"),
        	    BT::InputPort<unsigned long>("msg4_ul"),
        	    BT::InputPort<float>("msg4_f"),
        		BT::InputPort<std::string>("voice"),
        		BT::InputPort<std::string>("style"),
        		BT::InputPort<std::string>("rate"),
        		BT::InputPort<std::string>("pitch"),
        		BT::InputPort<std::string>("contour")};
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

    void ReplaceField(std::string &str, std::string place_holder, std::string replacement)
    {
		size_t pos = 0;
		pos = str.find(place_holder);
		if (pos != std::string::npos) {
			str = str.replace(pos, place_holder.length(), replacement);
		}
    }

    virtual BT::NodeStatus tick() override
    {
    	std::string voice = "en-US-JennyNeural";
    	std::string style = "chat";
    	std::string rate = "-15";
    	std::string pitch = "21";
    	std::string contour = "(0%, +0%) (100%, +0%)";

		std::string msg;
		std::string part;
		unsigned long part_ul;
		float part_f;

		std::stringstream m;
		std::stringstream port;

		for (int i = 0; i < 4; i++) {
			std::stringstream port;
			port << "msg" << (i + 1);
    		if (getInput<std::string>(port.str(), part)) {
				m << part << " ";
			}

    		if (getInput<unsigned long>(port.str() + "_ul", part_ul)) {
				m << part_ul << " ";
			}

    		if (getInput<float>(port.str() + "_f", part_f)) {
				m << part_f << " ";
			}
		}
		msg = m.str();
		
    	getInput<std::string>("voice", voice);
    	getInput<std::string>("style", style);
    	getInput<std::string>("rate", rate);
    	getInput<std::string>("pitch", pitch);
    	getInput<std::string>("contour", contour);

    	std::stringstream node_name;
    	{
    		const std::lock_guard<std::mutex> lock(_mutex);
    		node_name << "text_to_speech_action_client" << instance++;
    	}

        //node_ = rclcpp::Node::make_shared(node_name.str());
        auto action_client = rclcpp_action::create_client<speech_action_interfaces::action::Speak>(node_, "speak");
        // if no server is present, fail after 10 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            // RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        _aborted = false;

    	// Substitute placeholders
    	auto & settings = GameSettings::getInstance();
    	std::map<std::string, std::string> text_map;
    	if (settings.GetStringMapProperty("speech_sub_map", text_map)) {
    		// Fix - search for placeholders and then get replacement from map - simple for now
			for (auto const& item: text_map) {
		    	ReplaceAllOccurrences(item.first, item.second, msg);
			}
    	}

        // Unescape any SSML in the string(s)
        UnescapeXML(msg);

		std::cout << "Unescaped text: " << msg << std::endl;

		std::string ssml;
#if defined(READ_SSML_FILE)
    	std::ifstream file("./ssml.xml");
    	std::string ssml_in, str;
    	while (std::getline(file, str))
    	{
    		ssml += str;
    		ssml.push_back('\n');
    	}
#else
    	ssml = ssml_;
#endif

    	ReplaceField(ssml, "TEXT", msg);
    	ReplaceField(ssml, "VOICE", voice);
    	ReplaceField(ssml, "STYLE", style);
    	ReplaceField(ssml, "RATE", rate);
    	ReplaceField(ssml, "PITCH", pitch);
    	ReplaceField(ssml, "CONTOUR", contour);
//        RCLCPP_INFO(node_->get_logger(), "Sending text-to-speech cmd [%s]", msg.c_str());
        RCLCPP_INFO(node_->get_logger(), "Sending text-to-speech ssml cmd [%s]", ssml.c_str());

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
    std::string ssml_;
    bool _aborted;
    rclcpp::Node::SharedPtr node_;
    std::mutex _mutex;
    static int instance;
};

int TextToSpeechActionClient::instance = 0;


