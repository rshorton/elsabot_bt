#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include "behaviortree_cpp_v3/action_node.h"

#include "bt_custom_type_helpers.hpp"

#define TO_RAD(x) ((x)/180.0*M_PI)

#undef FUTURE_WAIT_BLOCK

class PickObjectAction : public BT::AsyncActionNode
{
public:
    PickObjectAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
		 _aborted(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("goal")
              };
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("pick_object_client");

        std::string goal_in;
        if (!getInput<std::string>("goal", goal_in)) {
            throw BT::RuntimeError("missing required input [goal]");
        }

        // Fix - add z to goal
        Pose2D goal = BT::convertFromString<Pose2D>(goal_in);

        // Fix - assumes z=0
        RCLCPP_INFO(node_->get_logger(), "Pick object goal %f %f %f", goal.x, goal.y, 0.0);

        moveit::planning_interface::MoveGroupInterface move_group(node_, "xarm");
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(0.10);
        move_group.setNumPlanningAttempts(10);
        move_group.setPlanningTime(5);
        move_group.setGoalTolerance(0.010);

        // Initial test using same procedure used by 'move_group_pick.cpp' of
        //  xarm_move_group_test package.

        //move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
        //move_group.move();

        tf2::Quaternion q;
        geometry_msgs::msg::Pose target;

        // Move to above pick position
        q.setRPY(TO_RAD(180.0), TO_RAD(0.0), TO_RAD(0.0));
        target.orientation = tf2::toMsg(q);

        target.position.x = -0.0;
        target.position.y = -0.157;
        target.position.z =  0.085;

        RCLCPP_INFO(node_->get_logger(), "Move to above object");
        move_group.setPoseTarget(target);
        move_group.move();

        // Open gripper
        RCLCPP_INFO(node_->get_logger(), "Open gripper");
        moveit::planning_interface::MoveGroupInterface move_group_eff(node_, "arm_end_effector");
        move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
        move_group_eff.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Move downward so that gripper is around object
        RCLCPP_INFO(node_->get_logger(), "Move down to object");
        geometry_msgs::msg::Pose target_grab = target;
        target_grab.position.z = target.position.z - 0.14;
        move_group.setPoseTarget(target_grab);
        move_group.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Close gripper
        RCLCPP_INFO(node_->get_logger(), "Close gripper");
        move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("closed"));
        move_group_eff.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Move to above drop point
        q.setRPY(TO_RAD(90.0), TO_RAD(0.0), TO_RAD(90.0));
        target.orientation = tf2::toMsg(q);

        target.position.x =  0.010;
        target.position.y =  0.000;
        target.position.z =  0.260;

        move_group.setPoseTarget(target);
        move_group.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Move to drop point
        RCLCPP_INFO(node_->get_logger(), "Move to drop location");

        target.position.x =  0.030;
        target.position.y =  0.000;
        target.position.z =  0.210;

        move_group.setPoseTarget(target);
        move_group.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Open gripper
        RCLCPP_INFO(node_->get_logger(), "Open gripper");
        move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
        move_group_eff.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Move to home  
        RCLCPP_INFO(node_->get_logger(), "Move to home location");
        move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
        move_group.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "finished");
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
