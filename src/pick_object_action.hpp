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
#include "ros_common.hpp"

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
        return{ BT::InputPort<std::string>("position")
              };
    }

    // FIX Refactor this into multiple actions
    virtual BT::NodeStatus tick() override
    {
        node_ = ROSCommon::GetInstance()->GetNode();
        //node_ = rclcpp::Node::make_shared("pick_object_client");

        std::string pos_in;
        if (!getInput<std::string>("position", pos_in)) {
            throw BT::RuntimeError("missing required input [position]");
        }

        Position pos = BT::convertFromString<Position>(pos_in);

        RCLCPP_INFO(node_->get_logger(), "Pick object goal %f %f %f", pos.x, pos.y, pos.z);

        if (pos.z < -0.160) {
            pos.z = -0.160;
            RCLCPP_INFO(node_->get_logger(), "Limiting object z to %f", pos.z);
        }

        moveit::planning_interface::MoveGroupInterface move_group(node_, "xarm");
        move_group.setMaxVelocityScalingFactor(20.0);
        move_group.setMaxAccelerationScalingFactor(0.10);
        move_group.setNumPlanningAttempts(10);
        move_group.setPlanningTime(5);
        move_group.setGoalTolerance(0.010);

        tf2::Quaternion q;
        geometry_msgs::msg::Pose target;

        // Move to above pick position
        q.setRPY(TO_RAD(180.0), TO_RAD(0.0), TO_RAD(0.0));
        target.orientation = tf2::toMsg(q);

        target.position.x = pos.x;
        target.position.y = pos.y;
        target.position.z = pos.z + 0.260;

        RCLCPP_INFO(node_->get_logger(), "Moving to above object");
        move_group.setPoseTarget(target);
        move_group.move();

        // Open gripper
        RCLCPP_INFO(node_->get_logger(), "Opening gripper");
        moveit::planning_interface::MoveGroupInterface move_group_eff(node_, "arm_end_effector");
        move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
        move_group_eff.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Move downward so that gripper is around object
        RCLCPP_INFO(node_->get_logger(), "Moving down to object");
        geometry_msgs::msg::Pose target_grab = target;
        target_grab.position.z = pos.z + 0.190;
        move_group.setPoseTarget(target_grab);
        move_group.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        // Close gripper
        RCLCPP_INFO(node_->get_logger(), "Closing gripper");
        move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("closed"));
        move_group_eff.move();

        if (_aborted) {
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "pick action finished");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
    	RCLCPP_INFO(node_->get_logger(), "requesting pick abort");
        _aborted = true;
    }
private:
    bool _aborted;
    rclcpp::Node::SharedPtr node_;
};
