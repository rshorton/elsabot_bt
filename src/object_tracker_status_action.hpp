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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_head_interfaces/msg/track_status.hpp"

#include <behaviortree_cpp_v3/action_node.h>

// Singleton for subscribing to tracker status message - shared by all ObjectTrackerStatusAction node instances
class ObjectTrackerStatusROSNodeIf
{
public:
	ObjectTrackerStatusROSNodeIf(ObjectTrackerStatusROSNodeIf const&) = delete;
	ObjectTrackerStatusROSNodeIf& operator=(ObjectTrackerStatusROSNodeIf const&) = delete;

    static std::shared_ptr<ObjectTrackerStatusROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<ObjectTrackerStatusROSNodeIf> s{new ObjectTrackerStatusROSNodeIf(node)};
        return s;
    }

    rclcpp::Node::SharedPtr node_;
    bool valid_;
    rclcpp::Subscription<robot_head_interfaces::msg::TrackStatus>::SharedPtr tracker_status_sub_;
    robot_head_interfaces::msg::TrackStatus tracker_status_;

private:
    ObjectTrackerStatusROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
		valid_(false)
	{
    	tracker_status_sub_ = node_->create_subscription<robot_head_interfaces::msg::TrackStatus>(
    	            "/head/tracked",
    				rclcpp::SystemDefaultsQoS(),
    				std::bind(&ObjectTrackerStatusROSNodeIf::statusCallback, this, std::placeholders::_1));
		RCLCPP_DEBUG(node_->get_logger(), "Sub to /head/tracker");
    }

    void statusCallback(robot_head_interfaces::msg::TrackStatus::SharedPtr msg)
    {
    	tracker_status_ = *msg;
    	valid_ = true;
		RCLCPP_DEBUG(node_->get_logger(), "Got tracked object status");
    }
};

class ObjectTrackerStatusAction : public BT::SyncActionNode
{
    public:
	ObjectTrackerStatusAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = ObjectTrackerStatusROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
			return{
				BT::InputPort<bool>("ck_state"),
				BT::InputPort<float>("min_duration"),
				BT::OutputPort<std::string>("position")};
        }

        virtual BT::NodeStatus tick() override
        {

        	if (!node_if_->valid_) {
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

			RCLCPP_DEBUG(node_if_->node_->get_logger(), "Tracking status: tracking [%d], ck [%d], duration [%f], ck [%f], id= [%d]",
					node_if_->tracker_status_.tracking,
					ck_state,
					node_if_->tracker_status_.duration,
					min_duration,
					node_if_->tracker_status_.object.id);

			// Success if specified state has been active for the specified duration
			if (ck_state == node_if_->tracker_status_.tracking &&
					node_if_->tracker_status_.duration > min_duration) {

				// Fix - position should include the frame id
				std::stringstream str;
        		str << node_if_->tracker_status_.x_ave << ","
					<< node_if_->tracker_status_.y_ave << ",0.0"
					<< std::endl;
				setOutput("position", str.str());
				return BT::NodeStatus::SUCCESS;
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<ObjectTrackerStatusROSNodeIf> node_if_;
};
