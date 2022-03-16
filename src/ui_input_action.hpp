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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include "ui_topics.hpp"
#include "ros_common.hpp"

class UIInputAction : public BT::SyncActionNode
{
public:
    UIInputAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("action"),
            BT::InputPort<std::string>("property"),
            BT::InputPort<std::string>("ck_value"),
            BT::OutputPort<std::string>("value")
        };
    }

    virtual BT::NodeStatus tick() override
    {
        std::string action;
        if (!getInput<std::string>("action", action)) {
            throw BT::RuntimeError("missing action");
        }

        std::string property;
        if (!getInput<std::string>("property", property)) {
            throw BT::RuntimeError("missing property");
        }

        std::string ck_value;
        bool ck_value_set = false;
        if (getInput<std::string>("ck_value", ck_value)) {
            ck_value_set = true;
        }

        UITopics *ui_topics = UITopics::GetInstance();
        if (!ui_topics) {
            return BT::NodeStatus::FAILURE;
        }

        rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

        std::string type;
        std::string value;
        bool updated = false;
        if (ui_topics->GetGeneric(property, type, value, updated)) {
            RCLCPP_DEBUG(node->get_logger(), "value: [%s], updated: [%d]", value.c_str(), updated);

            if (action == "ck_changed") {
                if (updated) {
                    if (ck_value_set) {
                        if (value == ck_value) {
                            return BT::NodeStatus::SUCCESS;
                        }
                    } else {
                        return BT::NodeStatus::SUCCESS;            
                    }
                }
            }                
        }
        return BT::NodeStatus::FAILURE;
    }
};
