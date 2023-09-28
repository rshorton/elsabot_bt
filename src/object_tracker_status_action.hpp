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

#include <stdio.h>
#include <sstream>
#include <string>
#include <limits>
#include <chrono>

#include "ros_common.hpp"
#include "robot_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_head_interfaces/msg/track_status.hpp"

#include <behaviortree_cpp_v3/action_node.h>

class ObjectTrackerStatusAction : public BT::SyncActionNode
{
    public:
	ObjectTrackerStatusAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

	static BT::PortsList providedPorts()
	{
		return{
			BT::InputPort<bool>("ck_state"),
			BT::InputPort<float>("min_duration"),
			BT::InputPort<bool>("fail_on_not_tracked"),
			BT::OutputPort<bool>("is_tracking"),
			BT::OutputPort<std::string>("position"),
			BT::OutputPort<std::shared_ptr<robot_head_interfaces::msg::TrackStatus>>("track_status")};
	}

	virtual BT::NodeStatus tick() override
	{
		rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

		robot_head_interfaces::msg::TrackStatus track_status;
		if (!RobotStatus::GetInstance()->GetTrackStatus(track_status)) {
			RCLCPP_WARN(node->get_logger(), "Track status not available");
			return BT::NodeStatus::FAILURE; 
		}

		bool ck_state;
		if (!getInput<bool>("ck_state", ck_state)) {
			throw BT::RuntimeError("missing ck_state");
		}

		float min_duration;
		if (!getInput<float>("min_duration", min_duration)) {
			throw BT::RuntimeError("missing min_duration");
		}

		// Original behavior is to return success only if tracking an object.
		// Allow success if no tracking to allow processing track status in all cases.
		bool fail_on_not_tracked = true;
		getInput<bool>("fail_on_not_tracked", fail_on_not_tracked);

		RCLCPP_DEBUG(node->get_logger(), "Tracking status: tracking [%d], ck [%d], duration [%f], ck [%f], id= [%d]",
				track_status.tracking,
				ck_state,
				track_status.duration,
				min_duration,
				track_status.object.id);

		// Success if specified state has been active for the specified duration
		bool is_tracking = ck_state == track_status.tracking &&	track_status.duration > min_duration;

		if (fail_on_not_tracked && !is_tracking) {
			return BT::NodeStatus::FAILURE;
		}

		std::stringstream str;
		str << track_status.object.position.point.x << ","
			<< track_status.object.position.point.y << ",0.0"
			<< std::endl;
		setOutput("position", str.str());
		setOutput("is_tracking", is_tracking);
		setOutput("track_status", std::make_shared<robot_head_interfaces::msg::TrackStatus>(std::move(track_status)));
		return BT::NodeStatus::SUCCESS;
	}
};
