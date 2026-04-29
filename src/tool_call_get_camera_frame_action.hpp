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

#include "ros_common.hpp"
#include "tool_call_data.hpp"

class ToolCallGetCameraFrameAction : public BT::SyncActionNode
{
public:
	ToolCallGetCameraFrameAction(const std::string& name, const BT::NodeConfig& config) :
        BT::SyncActionNode(name, config)
    {
        node_ = ROSCommon::GetInstance()->GetNode();        
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("command"),
                BT::InputPort<std::string>("base64_image"),
                BT::OutputPort<std::string>("result_json")};
    }


    BT::NodeStatus tick() override {
        std::string command;
        getInput<std::string>("command", command);

        // Report tool description on init
        if (command == "init" || command.empty()) {
            ToolCallData& tc_data = ToolCallData::getInstance();
            tc_data.declare_tool(tool_desc_);
            return BT::NodeStatus::SUCCESS;
        }

        std::string base64_image;
        if (!getInput<std::string>("base64_image", base64_image)) {
			throw BT::RuntimeError("missing base64_image");
        }
        RCLCPP_INFO(node_->get_logger(), "Tool call get camera frame, image size: %ld", base64_image.size());

        // FIX - this does not work!!!  How to pass image back in tool call result?
        std::string result_json;
        nlohmann::json result_obj = nlohmann::json::array();
        result_obj.push_back({{"type", "text"}, {"text", "Camera frame obtained."}});
        result_obj.push_back({{"type", "image_url"}, {"image_url", {{"url", base64_image}}}});

        result_json = result_obj.dump();
        setOutput("result_json", result_json);

        return BT::NodeStatus::SUCCESS;; 
    }

private:
    rclcpp::Node::SharedPtr node_;

    std::string tool_desc_ = R"({
        "type": "function",
        "function": {
            "name": "get_camera_frame",
            "description": "Gets a frame from the camera.  Returns a base64 encoded jpeg image."
        }
    })";
};
