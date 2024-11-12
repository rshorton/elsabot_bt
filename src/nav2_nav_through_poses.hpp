#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav2_util/geometry_utils.hpp"

#include "nav_utils.hpp"

#undef FUTURE_WAIT_BLOCK

class Nav2NavThroughPoses : public BT::ThreadedAction
{
    using pose_list = std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>;

public:
    Nav2NavThroughPoses(const std::string& name, const BT::NodeConfig& config)
        : BT::ThreadedAction(name, config),
		 _aborted(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
           	BT::InputPort<pose_list>("poses"),
           	BT::InputPort<bool>("calc_orientations"),
            BT::InputPort<std::string>("behavior_tree")
        };
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("nav2_nav_through_poses_client");
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(node_, "navigate_through_poses");

        // if no server is present, fail after N seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        // optional
        std::string behavior_tree;
        getInput<std::string>("behavior_tree", behavior_tree);

        bool calc_orientations = false;
        getInput<bool>("calc_orientations", calc_orientations);

        pose_list poses;
        if (!getInput<pose_list>("poses", poses)) {
            throw BT::RuntimeError("missing pose list");
        }

        RCLCPP_INFO(node_->get_logger(), "Navigating thru %ld poses", poses->size());
        
        if (calc_orientations) {
            calculate_pose_orientations(poses);
        }            

        nav2_msgs::action::NavigateThroughPoses::Goal goal_msg;
        goal_msg.poses = *poses;
        goal_msg.behavior_tree = behavior_tree;

        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 nav through poses send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 nav through poses goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(node_->get_logger(), "Waiting for result");

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

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult wrapped_result = result_future.get();

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
    rclcpp::Node::SharedPtr node_;
};
