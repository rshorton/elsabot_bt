// Action node that uses nav2 to drive a looping path.  It requires the
// custom nav2_fixed_path planner to plan a route based on predefined
// waypoints.  The first point is the nearest waypoint in front of the robot.
// The path then follows the predefined waypoints until the specified
// end position is reached.  The Regulated Pure Pursuit Nav controller
// was used.
//
// See the test tree bt_nav_loop.xml.

#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/geometry_utils.hpp"

#include "bt_custom_type_helpers.hpp"
#include "robot_status.hpp"

using nav2_util::geometry_utils::orientationAroundZAxis;
using namespace std::chrono_literals;

#undef FUTURE_WAIT_BLOCK

class Nav2ClientLoop : public BT::AsyncActionNode
{
private:
    enum class PositionState { MOVE_AWAY_FROM_GOAL, MOVE_TOWARD_GOAL};    

public:
    Nav2ClientLoop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
		 _aborted(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("loop_end_pos"),
                BT::InputPort<std::string>("loop_intermediate_pos"),
                BT::InputPort<std::string>("behavior_tree"),
        };
    }

    virtual BT::NodeStatus tick() override
    {
        PositionState position_state = PositionState::MOVE_AWAY_FROM_GOAL;

        node_ = rclcpp::Node::make_shared("nav2_client_loop");
        auto goal_update_publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_update", 1);

    	RobotStatus* robot_status = RobotStatus::GetInstance();
    	if (!robot_status) {
	    	RCLCPP_ERROR(node_->get_logger(), "Cannot get robot status for obtaining pose");
    		return BT::NodeStatus::FAILURE;
    	}

        auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
        // if no server is present, fail after 5 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        // optional
        std::string behavior_tree;
        getInput<std::string>("behavior_tree", behavior_tree);

        std::string loop_end_pos_str;
        if (!getInput<std::string>("loop_end_pos", loop_end_pos_str)) {
            throw BT::RuntimeError("missing required input [loop_end_pos]");
        }

        std::string loop_intermediate_pos_str;
        if (!getInput<std::string>("loop_intermediate_pos", loop_intermediate_pos_str)) {
            throw BT::RuntimeError("missing required input [loop_intermediate_pos]");
        }

        Pose2D loop_end_pos = BT::convertFromString<Pose2D>(loop_end_pos_str);
        Pose2D loop_intermediate_pos = BT::convertFromString<Pose2D>(loop_intermediate_pos_str);
        Pose2D goal;

        // Get the current state and determine whether the robot is currently at the loop end position.
        // If so, then set the initial goal to be at an intermeditate point along the loop.  Otherwise
        // make the initial goal the loop end point.
        {
            double x, y, z, yaw;

            bool got_pose = false;
        	for (int t = 0; t < 20; t++) {
                got_pose = robot_status->GetPose(x, y, z, yaw);
                if (got_pose) {
                    break;
                }
                RCLCPP_INFO(node_->get_logger(), "Robot pose not available.  Waiting...");
        		std::this_thread::sleep_for(100ms);
               	rclcpp::spin_some(node_);
        	}

            if (!got_pose) {
                RCLCPP_INFO(node_->get_logger(), "Robot pose not available.  Failing, cannot determine initial navigation goal.");
                return BT::NodeStatus::FAILURE;
            }

            if (calc_distance(x, y, loop_end_pos.x, loop_end_pos.y) > 2.0) {
                position_state = PositionState::MOVE_TOWARD_GOAL;
                goal = loop_end_pos;
            } else {
                position_state = PositionState::MOVE_AWAY_FROM_GOAL;
                goal = loop_intermediate_pos;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Goal %f %f %f", goal.x, goal.y, goal.yaw);

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

        auto temp_cnt = 0;

		RCLCPP_INFO(node_->get_logger(), "waiting for nav to complete...");

		while(true) {
        	auto result = rclcpp::spin_until_future_complete(node_, result_future, timeout);
        	if (result == rclcpp::FutureReturnCode::TIMEOUT) {
        		if (_aborted) {
                	RCLCPP_INFO(node_->get_logger(), "waiting for nav to complete...aborted");
        			break;
        		}

                if (++temp_cnt > 20) {
                    temp_cnt = 0;

                    double x, y, z, yaw;
                    if (!robot_status->GetPose(x, y, z, yaw)) {
                        RCLCPP_ERROR(node_->get_logger(), "Robot pose not available");
                        continue;
                    }

                    RCLCPP_INFO(node_->get_logger(), "current position: %f, %f", x, y);

                    if (position_state == PositionState::MOVE_AWAY_FROM_GOAL) {
                        auto dist_from_loop_end = calc_distance(x, y, loop_end_pos.x, loop_end_pos.y);
                        if (dist_from_loop_end > 2.0) {
                            position_state = PositionState::MOVE_TOWARD_GOAL;

                            geometry_msgs::msg::PoseStamped msg;
                            msg.header.frame_id = "map";
                            msg.header.stamp = node_->get_clock()->now();
                            msg.pose.position.x = loop_end_pos.x;
                            msg.pose.position.y = loop_end_pos.y;
                            msg.pose.position.z = 0.0;
                            goal_update_publisher->publish(msg);
                            RCLCPP_INFO(node_->get_logger(), "Published new goal for the loop end");
                        } else {
                            RCLCPP_INFO(node_->get_logger(), "Waiting to publish loop end goal (%f)", dist_from_loop_end);
                        }
                    }
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
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
    	RCLCPP_INFO(node_->get_logger(), "requesting nav abort");
        _aborted = true;
    }

private:
    double calc_distance(double x1, double y1, double x2, double y2) {
        return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    }

private:
    bool _aborted;
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};
