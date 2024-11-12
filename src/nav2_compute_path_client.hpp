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
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.h"

#include "nav2_compute_path_client_util.hpp"

using nav2_util::geometry_utils::orientationAroundZAxis;

struct Point
{
    double x, y;
};

namespace BT
{
template <> inline
Point convertFromString(StringView key)
{
    // three real numbers separated by commas
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 2)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
    	Point output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
		return output;
    }
}
} // end namespace BT

class Nav2ComputePathClient : public BT::ThreadedAction
{
public:
    Nav2ComputePathClient(const std::string& name, const BT::NodeConfig& config)
        : BT::ThreadedAction(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Point>("goal")};
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("nav2_compute_path_client");
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node_, "compute_path_to_pose");
        // if no server is present, fail after 5 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            // RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }
        // Take the goal from the InputPort of the Node
        Point goal;
        if (!getInput<Point>("goal", goal)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [goal]");
        }

        _aborted = false;

        RCLCPP_INFO(node_->get_logger(), "Sending goal %f %f", goal.x, goal.y);

        nav_msgs::msg::Path path;
        double len;
        if (GetPathToPose(node_, path, len, goal.x, goal.y)) {
            if (_aborted) {
                // this happens only if method halt() was invoked
                //_client.cancelAllGoals();
                RCLCPP_INFO(node_->get_logger(), "Aborted");
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "result received, path len= %f", len);
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_ERROR(node_->get_logger(), "Failed to get path to pose");
        return BT::NodeStatus::FAILURE;
    }

    virtual void halt() override {
        _aborted = true;
    }
private:
    bool _aborted;
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};
