#pragma once

#include <stdio.h>
#include <sstream>
#include <string>
#include <limits>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "face_control_interfaces/msg/track_status.hpp"

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

    void update()
    {
    	rclcpp::spin_some(node_);
    }

    rclcpp::Node::SharedPtr node_;
    bool valid_;
    rclcpp::Subscription<face_control_interfaces::msg::TrackStatus>::SharedPtr tracker_status_sub_;
    face_control_interfaces::msg::TrackStatus tracker_status_;

private:
    ObjectTrackerStatusROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
		valid_(false)
	{
    	tracker_status_sub_ = node_->create_subscription<face_control_interfaces::msg::TrackStatus>(
    	            "/head/tracked",
    				rclcpp::SystemDefaultsQoS(),
    				std::bind(&ObjectTrackerStatusROSNodeIf::statusCallback, this, std::placeholders::_1));
    }

    void statusCallback(face_control_interfaces::msg::TrackStatus::SharedPtr msg)
    {
    	tracker_status_ = *msg;
    	valid_ = true;
		RCLCPP_INFO(node_->get_logger(), "Got detected objects");
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
				BT::InputPort<float>("min_duration")};
        }

        virtual BT::NodeStatus tick() override
        {
        	node_if_->update();

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

			RCLCPP_INFO(node_if_->node_->get_logger(), "Tracking status: tracking [%d], ck [%d], duration [%f], ck [%f], id= [%d]",
					node_if_->tracker_status_.tracking,
					ck_state,
					node_if_->tracker_status_.duration,
					min_duration,
					node_if_->tracker_status_.object.id);

			// Success if specified state has been active for the specified duration
			if (ck_state == node_if_->tracker_status_.tracking &&
					node_if_->tracker_status_.duration > min_duration) {
				return BT::NodeStatus::SUCCESS;
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<ObjectTrackerStatusROSNodeIf> node_if_;
};
