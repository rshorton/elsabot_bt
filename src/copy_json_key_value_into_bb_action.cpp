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

// FIX - consider supporting value validation also.

namespace
{
    const std::string AS_TYPE_STRING = "string";
    const std::string AS_TYPE_BOOL = "bool";
    const std::string AS_TYPE_INT = "int";
    const std::string AS_TYPE_UINT = "uint";
    const std::string AS_TYPE_DOUBLE = "double";
}

BT::PortsList CopyJsonKeyValueIntoBBAction::providedPorts() {
    return {BT::InputPort<std::string>("json_in"),              // Json to read
            BT::InputPort<std::string>("json_key_pointer"),     // Json pointer for key value to be copied
            BT::InputPort<std::string>("as_type"),              // The expected type
            BT::OutputPort<std::string>("str_output"),          // BB variable to receive json value
            BT::OutputPort<bool>("bool_output"),
            BT::OutputPort<int>("int_output"),
            BT::OutputPort<unsigned int>("uint_output"),
            BT::OutputPort<double>("double_output")
           };
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
        auto val = j.at(p);
        std::string val_str;

        // Anything can be represented as a string
        if (as_type == AS_TYPE_STRING) {
            if (val.is_object() || val.is_array()) {
                val_str = val.dump();
            } else if (val.is_string()) {
                val_str = val.get<std::string>();
            } else {
                std::stringstream ss;
                ss << val;
                val_str = ss.str();
            }                 
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: %s returning as string", name().c_str(), json_key_pointer.c_str());
            setOutput("str_output", val_str);

        } else if (as_type == AS_TYPE_BOOL) {
            if (!val.is_boolean()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s: %s is not a bool but was requested as a bool",
                            name().c_str(), json_key_pointer.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: %s returning as bool", name().c_str(), json_key_pointer.c_str());
            setOutput("bool_output", (bool)val);

        } else if (as_type == AS_TYPE_DOUBLE) {
            if (!val.is_number()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s: %s is not a number but was requested as a double",
                            name().c_str(), json_key_pointer.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: %s returning as double", name().c_str(), json_key_pointer.c_str());
            setOutput("double_output", (double)val);

        } else if (as_type == AS_TYPE_INT) {
            if (!val.is_number()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s: %s is not a number but was requested as an int",
                            name().c_str(), json_key_pointer.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: %s returning as int", name().c_str(), json_key_pointer.c_str());
            setOutput("int_output", (int)val);

        } else if (as_type == AS_TYPE_UINT) {
            if (!val.is_number()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s: %s is not a number but was requested as an Uint",
                            name().c_str(), json_key_pointer.c_str());
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: %s returning as Uint", name().c_str(), json_key_pointer.c_str());
            setOutput("uint_output", (unsigned int)val);

        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: the requested type %s is invalid", name().c_str(), as_type.c_str());
            return BT::NodeStatus::FAILURE;
        }

    } catch (const json::exception& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get specified json pointer from json %s, at: %s",
            json_key_pointer.c_str(), ex.what());
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CopyJsonKeyValueIntoBBAction::onRunning() {
    return BT::NodeStatus::SUCCESS;
}

void CopyJsonKeyValueIntoBBAction::onHalted() {
}
