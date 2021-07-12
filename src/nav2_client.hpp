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
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/geometry_utils.hpp"

using nav2_util::geometry_utils::orientationAroundZAxis;

#undef FUTURE_WAIT_BLOCK

// Custom type
struct Pose2D
{
    double x, y;
    double yaw;
};

namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by commas
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
		output.yaw = convertFromString<double>(parts[2]);
		return output;
    }
}
} // end namespace BT

class Nav2Client : public BT::AsyncActionNode
{
public:
    Nav2Client(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
		 _aborted(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("goal"), BT::InputPort<std::string>("behavior_tree")};
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("nav2_client");
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
        // if no server is present, fail after 5 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            // RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        // optional
        std::string behavior_tree;
        getInput<std::string>("behavior_tree", behavior_tree);

        std::string goal_in;
        if (!getInput<std::string>("goal", goal_in)) {
            throw BT::RuntimeError("missing required input [goal]");
        }

        Pose2D goal = BT::convertFromString<Pose2D>(goal_in);

        RCLCPP_INFO(node_->get_logger(), "Sending goal %f %f %f", goal.x, goal.y, goal.yaw);

        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->get_clock()->now();
        goal_msg.pose.pose.position.x = goal.x;
        goal_msg.pose.pose.position.y = goal.y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation = orientationAroundZAxis(goal.yaw/180.0*M_PI);
        goal_msg.behavior_tree = behavior_tree;

        RCLCPP_INFO(node_->get_logger(), "orientation %f %f %f %f", goal_msg.pose.pose.orientation.x, goal_msg.pose.pose.orientation.y, goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w);

        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(node_->get_logger(), "Waiting for result");
#if defined(FUTURE_WAIT_BLOCK)
        if (rclcpp::spin_until_future_complete(node_, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
            return BT::NodeStatus::FAILURE;
        }
#else
        bool bSuccess = false;
		auto timeout = std::chrono::duration<float, std::milli>(250);

		RCLCPP_INFO(node_->get_logger(), "waiting for nav to complete...");

		while(true) {
        	auto result = rclcpp::spin_until_future_complete(node_, result_future, timeout);
        	if (result == rclcpp::FutureReturnCode::TIMEOUT) {
        		if (_aborted) {
                	RCLCPP_INFO(node_->get_logger(), "waiting for nav to complete...aborted");
        			break;
        		}
        	} else {
        		if (result != rclcpp::FutureReturnCode::SUCCESS) {
        			RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
        			return BT::NodeStatus::FAILURE;
        		}
        		bSuccess = true;
            	RCLCPP_INFO(node_->get_logger(), "waiting for nav to complete...success");
            	break;
        	}
        }

        if (!bSuccess && _aborted) {
        	_aborted = false;
        	RCLCPP_INFO(node_->get_logger(), "canceling nav after abort");

        	auto future_cancel = action_client->async_cancel_goal(goal_handle);
    	    if (rclcpp::spin_until_future_complete(node_, future_cancel) !=
    	    	rclcpp::FutureReturnCode::SUCCESS) {
    	    	RCLCPP_ERROR(node_->get_logger(), "failed to cancel nav goal");
    	    } else {
    	    	RCLCPP_INFO(node_->get_logger(), "nav goal is being canceled");
    	    }
        	return BT::NodeStatus::IDLE;
        }

#endif
        if (_aborted) {
        	_aborted = false;
            // this happens only if method halt() was invoked
            //_client.cancelAllGoals();
            RCLCPP_INFO(node_->get_logger(), "Nav aborted");
            return BT::NodeStatus::IDLE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                return BT::NodeStatus::FAILURE;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                return BT::NodeStatus::FAILURE;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                return BT::NodeStatus::FAILURE;
        }


        RCLCPP_INFO(node_->get_logger(), "result received");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
    	RCLCPP_INFO(node_->get_logger(), "requesting nav abort");
        _aborted = true;
    }
private:
    bool _aborted;
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};
