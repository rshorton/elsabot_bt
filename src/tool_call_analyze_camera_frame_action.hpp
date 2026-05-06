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

#include <behaviortree_cpp/action_node.h>

#include "ai_session.hpp"
#include "ros_common.hpp"
#include "tool_call_data.hpp"

#define VLLM_GEMMA

class ToolCallAnalyzeCameraFrameAction : public BT::StatefulActionNode
{
public:
	  ToolCallAnalyzeCameraFrameAction(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {
        node_ = ROSCommon::GetInstance()->GetNode();        
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("command"),
                BT::InputPort<std::string>("args_json"),
                BT::InputPort<std::string>("base64_image"),
                BT::OutputPort<std::string>("result_json")};
    }

    BT::NodeStatus onStart() {
        //auto data_callback = [&](const std::string &data) {
        //};

        std::string command;
        getInput<std::string>("command", command);

        // Report tool description on init
        if (command == "init" || command.empty()) {
            ToolCallData& tc_data = ToolCallData::getInstance();
            tc_data.declare_tool(tool_desc_);
            return BT::NodeStatus::SUCCESS;
        }

        std::string args_json;
        if (!getInput<std::string>("args_json", args_json)) {
      			throw BT::RuntimeError("missing args_json");
        }

        std::string base64_image;
        if (!getInput<std::string>("base64_image", base64_image)) {
      			throw BT::RuntimeError("missing base64_image");
        }

        std::string system_prompt = "You analyze images.";
        ai_session_ = std::make_unique<AISession>(model_, max_context_size_, auth_token_,
                                                  host_address_and_port_, resource_,
                                                  timeout_ms_, system_prompt, node_->get_logger(),
                                                  nullptr);
        if (!ai_session_) {
            RCLCPP_ERROR(node_->get_logger(), "ToolCallAnalyzeCameraFrameAction, failed to create an AI session object");
            return BT::NodeStatus::FAILURE;
        }                                                  

        nlohmann::json args;
        try {
          args = nlohmann::json::parse(args_json);
        } catch (nlohmann::json::parse_error& ex) {
          RCLCPP_ERROR(node_->get_logger(), "ToolCallAnalyzeCameraFrameAction, failed parsing tool call args %s, at: %ld",
                       args_json.c_str(), ex.byte);
          return BT::NodeStatus::FAILURE;
        }
        
        std::string prompt = args["prompt"];
        RCLCPP_INFO(node_->get_logger(), "ToolCallAnalyzeCameraFrameAction, prompt: %s", prompt.c_str());
        ai_session_->user_prompt(prompt, false, "", base64_image);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        std::string full_response;
        bool is_tool_call;
        AISession::Result result;
        auto finished = ai_session_->is_finished(full_response, is_tool_call, result);

        if (finished) {
            RCLCPP_INFO(node_->get_logger(), "ToolCallAnalyzeCameraFrameAction VLM finished, result: %s",
                         AISession::result_to_str(result).c_str());

            if (result == AISession::Result::cancelled ||
                result == AISession::Result::timeout ||
                result == AISession::Result::failed) {
                return BT::NodeStatus::FAILURE;
            } else {
                nlohmann::json result_obj = {{"result", full_response}};
                auto result = result_obj.dump();

                RCLCPP_INFO(node_->get_logger(), "ToolCallAnalyzeCameraFrameAction result: %s", result.c_str());
                setOutput("result_json", result);
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() {
        RCLCPP_INFO(node_->get_logger(), "ToolCallAnalyzeCameraFrameAction halted");
        if (ai_session_) {
            ai_session_->cancel();
        }
    }

private:
    std::string tool_desc_ = R"({
        "type": "function",
        "function": {
            "name": "analyze_camera_frame",
            "description": "Gets a frame from the camera and runs VLM processing.",
            "parameters": {
                "type": "object",
                "properties": {
                    "prompt": {
                        "type": "string",
                        "description": "The prompt to use when analyzing the image using VLM"
                    }
                },
                "required": ["prompt"]                
            }
        }
    })";

    rclcpp::Node::SharedPtr node_;

    std::string model_{"cyankiwi/gemma-4-26B-A4B-it-AWQ-4bit"};
    std::string resource_{"/v1/chat/completions"};
    int max_context_size_{64000};
    std::string auth_token_{"none"};

    std::string host_address_and_port_{"http://localhost:8000"};
    int timeout_ms_ = 8000;

    std::unique_ptr<AISession> ai_session_;
};
