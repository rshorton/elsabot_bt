#pragma once

#include <stdio.h>
#include <sstream>
#include <string>
#include <limits>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <behaviortree_cpp_v3/action_node.h>

// Singleton for subscribing to object location message - shared by all ObjectLocationStatusAction node instances
class ObjectLocationStatusROSNodeIf
{
public:
	ObjectLocationStatusROSNodeIf(ObjectLocationStatusROSNodeIf const&) = delete;
	ObjectLocationStatusROSNodeIf& operator=(ObjectLocationStatusROSNodeIf const&) = delete;

    static std::shared_ptr<ObjectLocationStatusROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<ObjectLocationStatusROSNodeIf> s{new ObjectLocationStatusROSNodeIf(node)};
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
    ObjectLocationStatusROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
		valid_(false),
		last_time_(std::chrono::high_resolution_clock::now())
	{
    	// Using the goal_update topic for this which is published by the clicked_point_to_pose node
    	location_status_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    	            "goal_update",
    				rclcpp::SystemDefaultsQoS(),
    				std::bind(&ObjectLocationStatusROSNodeIf::statusCallback, this, std::placeholders::_1));
    }

    void statusCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
    	location_status_ = *msg;
    	valid_ = true;
    	last_time_ = std::chrono::high_resolution_clock::now();
		//RCLCPP_INFO(node_->get_logger(), "Got detected objects");
    }
};

class ObjectLocationStatusAction : public BT::SyncActionNode
{
    public:
	ObjectLocationStatusAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = ObjectLocationStatusROSNodeIf::instance(node);
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
        std::shared_ptr<ObjectLocationStatusROSNodeIf> node_if_;
};
