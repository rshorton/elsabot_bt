#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

class Nav2ClearLocalCostMap : public BT::SyncActionNode
{
public:
    Nav2ClearLocalCostMap(const std::string& name, const BT::NodeConfiguration& config)
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

        auto client = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap");

        // if no server is present, fail after 5 seconds
        if (!client->wait_for_service(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "Action service not available after waiting");
            return BT::NodeStatus::FAILURE;
        }
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

        RCLCPP_INFO(node_->get_logger(), "Sending request to clear local cost map");

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "completed");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to clear local cost map");
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};
