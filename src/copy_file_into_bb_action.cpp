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

#include <stdio.h>
#include <string>

#include <fstream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/action_node.h>

#include "copy_file_into_bb_action.hpp"

using json = nlohmann::json;

BT::PortsList CopyFileIntoBBAction::providedPorts() {
    return {BT::InputPort<std::string>("file_path"),
            BT::OutputPort<std::string>("port")
           };
}

BT::NodeStatus CopyFileIntoBBAction::onStart() {
    std::string file_path;
    if (!getInput<std::string>("file_path", file_path)) {
        throw BT::RuntimeError("missing file_path");
    }

    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file %s", file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    setOutput("port", buffer.str());
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CopyFileIntoBBAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void CopyFileIntoBBAction::onHalted() {
}
