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
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

class Nav2ClearGlobalCostMap : public BT::SyncActionNode
{
public:
    Nav2ClearGlobalCostMap(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
    	return{};
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("nav2_clear_cost_map");

        auto client = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");

        // if no server is present, fail after 5 seconds
        if (!client->wait_for_service(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "Action service not available after waiting");
            return BT::NodeStatus::FAILURE;
        }
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

        RCLCPP_INFO(node_->get_logger(), "Sending request to clear global cost map");

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "completed");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to clear global cost map");
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};
