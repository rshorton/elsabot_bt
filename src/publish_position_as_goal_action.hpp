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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "bt_custom_type_helpers.hpp"
#include "ros_common.hpp"


// Singleton for publishing the pose update - shared by all PublishPositionAsGoalAction instances
class PublishPositionAsGoalActionROSNodeIf
{
public:
	PublishPositionAsGoalActionROSNodeIf(PublishPositionAsGoalActionROSNodeIf const&) = delete;
	PublishPositionAsGoalActionROSNodeIf& operator=(PublishPositionAsGoalActionROSNodeIf const&) = delete;

    static std::shared_ptr<PublishPositionAsGoalActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
        static std::shared_ptr<PublishPositionAsGoalActionROSNodeIf> s{new PublishPositionAsGoalActionROSNodeIf(node)};
        return s;
    }

    void pubGoalUpdate(const Position &position)
    {
    	RCLCPP_INFO(node_->get_logger(), "Pub goal update: x,y= %f, %f", position.x, position.y);
    	auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.frame_id = "oakd_center_camera";
        msg.header.stamp = node_->get_clock()->now();
        msg.pose.position.x = position.x/1000.0;
        msg.pose.position.y = position.y/1000.0;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;
        goal_update_publisher_->publish(msg);
    }

private:
    PublishPositionAsGoalActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
    	goal_update_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_update", 1);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_update_publisher_;
};

class PublishPositionAsGoalAction : public BT::SyncActionNode
{
    public:
		PublishPositionAsGoalAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = PublishPositionAsGoalActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("position")};
        }

        virtual BT::NodeStatus tick() override
        {
			std::string position_str;					
			if (!getInput<std::string>("position", position_str)) {
				throw BT::RuntimeError("missing position");
			}
			node_if_->pubGoalUpdate(BT::convertFromString<Position>(position_str));
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<PublishPositionAsGoalActionROSNodeIf> node_if_;
};
