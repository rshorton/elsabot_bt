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
#include "robot_status.hpp"

class ToolCallGetPositionAction : public BT::SyncActionNode
{
public:
	ToolCallGetPositionAction(const std::string& name, const BT::NodeConfig& config) :
        BT::SyncActionNode(name, config)
    {
        node_ = ROSCommon::GetInstance()->GetNode();        
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("command"),
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

       	RobotStatus* robot_status = RobotStatus::GetInstance();
    	if (!robot_status) {
	    	RCLCPP_ERROR(node_->get_logger(), "Cannot get robot status for obtaining pose");
            set_failure_result();
    		return BT::NodeStatus::FAILURE;
    	}

        double x, y, z, yaw = 0.0;
        auto valid_pose = false;

        for (int t = 0; t < 20; t++) {
            valid_pose = robot_status->GetPose(x, y, z, yaw);
            if (valid_pose) {
                break;
            }
        }
        
        if (!valid_pose) {
            set_failure_result();
            return BT::NodeStatus::SUCCESS;
        }

        yaw = yaw*180.0f/M_PI;

        nlohmann::json result_obj;
        result_obj["x"] = x;
        result_obj["y"] = y;
        result_obj["yaw"] = yaw;

        std::string result_json = result_obj.dump();
        setOutput("result_json", result_json);
        RCLCPP_INFO(node_->get_logger(), "Tool call get position, result: %s", result_json.c_str());
        return BT::NodeStatus::SUCCESS;; 
    }

private:
    void set_failure_result() {
        setOutput("result_json", R"("result": "failed")");
    }

    rclcpp::Node::SharedPtr node_;

    std::string tool_desc_ = R"({
        "type": "function",
        "function": {
            "name": "get_robot_position",
            "description": "Gets the current position and orientation of the robot. Use this tool when you are asked for or need your position. Returns a json object containing, x: x position in meters, y: y position in meters, yaw: orientation in degrees"
        }
    })";
};
