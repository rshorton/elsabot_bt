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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <behaviortree_cpp_v3/action_node.h>

// Singleton for subscribing to object location message - shared by all ObjectTrackerLocationStatusAction node instances
class ObjectTrackerLocationStatusROSNodeIf
{
public:
	ObjectTrackerLocationStatusROSNodeIf(ObjectTrackerLocationStatusROSNodeIf const&) = delete;
	ObjectTrackerLocationStatusROSNodeIf& operator=(ObjectTrackerLocationStatusROSNodeIf const&) = delete;

    static std::shared_ptr<ObjectTrackerLocationStatusROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<ObjectTrackerLocationStatusROSNodeIf> s{new ObjectTrackerLocationStatusROSNodeIf(node)};
        return s;
    }

    void update()
    {
    	rclcpp::spin_some(node_);
    }

    rclcpp::Node::SharedPtr node_;
    bool valid_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_status_sub_;
    geometry_msgs::msg::PoseStamped location_status_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;

private:
    ObjectTrackerLocationStatusROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
		valid_(false),
		last_time_(std::chrono::high_resolution_clock::now())
	{
    	// Using the goal_update topic for this which is published by the clicked_point_to_pose node
    	location_status_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    	            "/head/tracked_mapped",
    				rclcpp::SystemDefaultsQoS(),
    				std::bind(&ObjectTrackerLocationStatusROSNodeIf::statusCallback, this, std::placeholders::_1));
    }

    void statusCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
    	location_status_ = *msg;
    	valid_ = true;
    	last_time_ = std::chrono::high_resolution_clock::now();
		//RCLCPP_INFO(node_->get_logger(), "Got detected objects");
    }
};

class ObjectTrackerLocationStatusAction : public BT::SyncActionNode
{
    public:
	ObjectTrackerLocationStatusAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = ObjectTrackerLocationStatusROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
			return{
				BT::OutputPort<std::string>("x"),
				BT::OutputPort<std::string>("y") };
        }

        // Failure when no new position update is available
        virtual BT::NodeStatus tick() override
        {
        	node_if_->update();

			auto now = std::chrono::high_resolution_clock::now();
        	auto elapsed = now - node_if_->last_time_;
			node_if_->last_time_ = now;
			typedef std::chrono::duration<float> float_seconds;
			auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);
			// pose updated at 1Hz
			if (seconds.count() > 1.5) {
				node_if_->valid_ = false;
			}

			RCLCPP_INFO(node_if_->node_->get_logger(), "Location status: valid [%d], x [%f], y [%f]",
					node_if_->valid_,
					node_if_->location_status_.pose.position.x,
					node_if_->location_status_.pose.position.y);


			if (node_if_->valid_) {
				setOutput("x", std::to_string(node_if_->location_status_.pose.position.x));
				setOutput("y", std::to_string(node_if_->location_status_.pose.position.y));
				return BT::NodeStatus::SUCCESS;
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<ObjectTrackerLocationStatusROSNodeIf> node_if_;
};
