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
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <behaviortree_cpp/action_node.h>

#include "copy_json_key_value_into_bb_action.hpp"

using json = nlohmann::json;

BT::PortsList CopyJsonKeyValueIntoBBAction::providedPorts() {
    return {BT::InputPort<std::string>("json_in"),              // Json to read
            BT::InputPort<std::string>("json_key_pointer"),     // Json pointer for key value to be copied
            BT::OutputPort<std::string>("output")};             // BB variable to receive json value
}

BT::NodeStatus CopyJsonKeyValueIntoBBAction::onStart() {
    std::string json_in;
    if (!getInput<std::string>("json_in", json_in)) {
        throw BT::RuntimeError("missing json_in");
    }

    std::string json_key_pointer;
    if (!getInput<std::string>("json_key_pointer", json_key_pointer)) {
        throw BT::RuntimeError("missing json_key_pointer");
    }

    json j;
    try {
        j = json::parse(json_in);
    } catch (json::parse_error& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse json %s, at: %ld", json_in.c_str(), ex.byte);
        return BT::NodeStatus::FAILURE;
    }

    try {
        json::json_pointer p(json_key_pointer);
        auto val = j.at(p);
        std::string val_str;

        // If an object or array, return json
        if (val.is_object() || val.is_array()) {
            val_str = val.dump();
        } else {
            std::stringstream ss;
            ss << val;
            val_str = ss.str();
        }

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), val_str.c_str());

        setOutput("output", val_str);

    } catch (json::parse_error& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get specified json pointer from json %s, at: %ld",
            json_key_pointer.c_str(), ex.byte);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CopyJsonKeyValueIntoBBAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void CopyJsonKeyValueIntoBBAction::onHalted() {
}
