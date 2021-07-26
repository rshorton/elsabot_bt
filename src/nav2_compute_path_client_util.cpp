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

#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_compute_path_client_util.hpp"

using nav2_util::geometry_utils::orientationAroundZAxis;

bool GetPathToPose(rclcpp::Node::SharedPtr node, nav_msgs::msg::Path &path, double &len, double x, double y)
{
	auto action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node, "compute_path_to_pose");
	// if no server is present, fail after 5 seconds
	if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
		RCLCPP_ERROR(node->get_logger(), "compute_path_to_pose action server not available after waiting");
		return false;
	}

	nav2_msgs::action::ComputePathToPose::Goal goal_msg;
	goal_msg.goal.header.frame_id = "map";
	goal_msg.goal.header.stamp = node->get_clock()->now();
	goal_msg.goal.pose.position.x = x;
	goal_msg.goal.pose.position.y = y;
	goal_msg.goal.pose.position.z = 0.0;
	goal_msg.goal.pose.orientation = orientationAroundZAxis(0.0);
	goal_msg.planner_id="GridBased";
	goal_msg.use_start = false;

	auto goal_handle_future = action_client->async_send_goal(goal_msg);
	if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node->get_logger(), "send goal call failed");
		return false;
	}

	rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr goal_handle = goal_handle_future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
		return false;
	}

	auto result_future = action_client->async_get_result(goal_handle);

	RCLCPP_INFO(node->get_logger(), "Waiting for result");
	if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node->get_logger(), "get result call failed " );
		return false;
	}

	rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult wrapped_result = result_future.get();

	switch (wrapped_result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
			return false;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
			return false;
		default:
			RCLCPP_ERROR(node->get_logger(), "Unknown result code");
			return false;
	}

	RCLCPP_INFO(node->get_logger(), "result received");

	path = wrapped_result.result->path;
	len = nav2_util::geometry_utils::calculate_path_length(wrapped_result.result->path, 0);
	RCLCPP_INFO(node->get_logger(), "Path length: %f", len);
	return true;
}
