/*
Copyright 2023 Scott Horton

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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/action_node.h>

#include "robot_head_interfaces/msg/track_status.hpp"
#include "robot_cat_game.hpp"

using namespace std;

class RobotCatGameNextPoseAction : public BT::SyncActionNode
{
    public:

	RobotCatGameNextPoseAction(const std::string& name, const BT::NodeConfig& config)
		: BT::SyncActionNode(name, config)
	{
	}

	static BT::PortsList providedPorts()
	{
		return{ BT::InputPort<bool>("cat_tracked"),
				BT::InputPort<std::shared_ptr<robot_head_interfaces::msg::TrackStatus>>("track_status"),
				BT::OutputPort<std::string>("pose"),
				BT::OutputPort<std::string>("action_desc") };
	}

	virtual BT::NodeStatus tick() override
	{
		bool cat_tracked = false;
		if (!getInput<bool>("cat_tracked", cat_tracked)) {
			throw BT::RuntimeError("missing cat_tracked");
		}

		std::shared_ptr<robot_head_interfaces::msg::TrackStatus> track_status;

#if 0 	// Use this when BT >= 4.2.  This will provide thread-safe access to the shared ptr.
		// Not essential since the nodes currently used run in the main thread of the BT process.
		if (auto any_locked = BT::Blackboard::getLockedPortContent("track_status")) {
			track_status = *any_locked.get()->cast<std::shared_ptr<robot_head_interfaces::msg::TrackStatus>>();
		} else {
			throw BT::RuntimeError("missing track_status");
		}
#else
		if (!getInput<std::shared_ptr<robot_head_interfaces::msg::TrackStatus>>("track_status", track_status)) {
			throw BT::RuntimeError("missing track_status");
		}
#endif

		auto game = RobotCatGame::GetRobotCatGame();
		if (game) {
			OrientationRPY pose;
			std::string action_desc;

			if (game->NextHeadPose(*track_status, pose, action_desc)) {
				setOutput("pose", convertToString(pose));
				setOutput("action_desc", action_desc);
				return BT::NodeStatus::SUCCESS;
			}					
		}
		return BT::NodeStatus::FAILURE;
	}
};
