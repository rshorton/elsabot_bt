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

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_head_interfaces/msg/head_pose.hpp"
#include <behaviortree_cpp/action_node.h>

// Singleton for publishing the Track control state - shared by all TrackManualActionNode instances
class TrackManualActionROSNodeIf
{
public:
	TrackManualActionROSNodeIf(TrackManualActionROSNodeIf const&) = delete;
	TrackManualActionROSNodeIf& operator=(TrackManualActionROSNodeIf const&) = delete;

    static std::shared_ptr<TrackManualActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
        static std::shared_ptr<TrackManualActionROSNodeIf> s{new TrackManualActionROSNodeIf(node)};
        return s;
    }

    void setHeadPose(const OrientationRPY &orient)
    {
    	RCLCPP_DEBUG(node_->get_logger(), "Set head pose: roll= %f, pitch= %f, yaw= %f",
    				orient.r, orient.p, orient.y);

    	auto msg = robot_head_interfaces::msg::HeadPose();
		msg.roll = orient.r;
		msg.pitch = orient.p;
		msg.yaw = orient.y;
        head_pose_pub_->publish(msg);
    }

private:
    TrackManualActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
    	head_pose_pub_ = node_->create_publisher<robot_head_interfaces::msg::HeadPose>("/head/pose", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::HeadPose>::SharedPtr head_pose_pub_;
};

class TrackManualAction : public BT::SyncActionNode
{
public:
	TrackManualAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
		: BT::SyncActionNode(name, config)
	{
		node_if_ = TrackManualActionROSNodeIf::instance(node);
	}

	static BT::PortsList providedPorts()
	{
		return { BT::InputPort<std::string>("pose") };
	}

	virtual BT::NodeStatus tick() override
	{
		std::string pose_str;
		if (!getInput<std::string>("pose", pose_str)) {
			throw BT::RuntimeError("missing pose_str");
		}

		OrientationRPY orient = BT::convertFromString<OrientationRPY>(pose_str);
		node_if_->setHeadPose(orient);
		return BT::NodeStatus::SUCCESS;
	}

private:
	std::shared_ptr<TrackManualActionROSNodeIf> node_if_;
};
