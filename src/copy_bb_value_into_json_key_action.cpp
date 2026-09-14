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

#include "copy_bb_value_into_json_key_action.hpp"

using json = nlohmann::json;

namespace
{
    const std::string AS_TYPE_STRING = "string";
    const std::string AS_TYPE_BOOL = "bool";
    const std::string AS_TYPE_INT = "int";
    const std::string AS_TYPE_UINT = "uint";
    const std::string AS_TYPE_DOUBLE = "double";
}

BT::PortsList CopyBBValueIntoJsonKeyValueAction::providedPorts() {
    return {BT::BidirectionalPort<std::string>("json_in_out"),  // Json to read
            BT::InputPort<std::string>("json_key_pointer"),     // Json pointer for key value to be set
            BT::InputPort<std::string>("as_type"),              // The expected type
            BT::InputPort<std::string>("str_value"),            // BB variable to read and set into json
            BT::InputPort<bool>("bool_value"),                  
            BT::InputPort<double>("double_value"),
            BT::InputPort<int>("int_value"),
            BT::InputPort<unsigned int>("uint_value")};
}

BT::NodeStatus CopyBBValueIntoJsonKeyValueAction::onStart() {
    std::string json_in;
    if (!getInput<std::string>("json_in_out", json_in)) {
        throw BT::RuntimeError("missing json_in_out");
    }

    std::string json_key_pointer;
    if (!getInput<std::string>("json_key_pointer", json_key_pointer)) {
        throw BT::RuntimeError("missing json_key_pointer");
    }

    std::string as_type = "string";
    getInput<std::string>("as_type", as_type);

    json j;
    try {
        j = json::parse(json_in);
    } catch (json::parse_error& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse json %s, at: %ld", json_in.c_str(), ex.byte);
        return BT::NodeStatus::FAILURE;
    }

    try {
        json::json_pointer p(json_key_pointer);

        if (as_type == AS_TYPE_STRING) {
            std::string value;
            getInput<std::string>("str_value", value);
            j[p] = value;

        } else if (as_type == AS_TYPE_BOOL) {
            bool value;
            getInput<bool>("bool_value", value);
            j[p] = value;

        } else if (as_type == AS_TYPE_DOUBLE) {
            double value;
            getInput<double>("double_value", value);
            j[p] = value;

        } else if (as_type == AS_TYPE_INT) {
            int value;
            getInput<int>("int_value", value);
            j[p] = value;

        } else if (as_type == AS_TYPE_UINT) {
            unsigned int value;
            getInput<unsigned int>("uint_value", value);
            j[p] = value;

        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: the requested type %s is invalid", name().c_str(), as_type.c_str());
            return BT::NodeStatus::FAILURE;
        }

        std::string json_out = j.dump();

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), json_out.c_str());
        setOutput("json_in_out", json_out);

    } catch (const json::exception& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set specified json pointer into json %s, at: %s",
            json_key_pointer.c_str(), ex.what());
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CopyBBValueIntoJsonKeyValueAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void CopyBBValueIntoJsonKeyValueAction::onHalted() {
}
