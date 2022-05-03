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

class SetGripperPositionAction : public BT::AsyncActionNode
{
public:
    SetGripperPositionAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
		 _aborted(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("position")
              };
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = ROSCommon::GetInstance()->GetNode();
     
        std::string pos;
        if (!getInput<std::string>("position", pos)) {
            throw BT::RuntimeError("missing required input [position]");
        }

        RCLCPP_INFO(node_->get_logger(), "Gripper position goal %s", pos.c_str());

        moveit::planning_interface::MoveGroupInterface move_group(node_, "xarm");
        move_group.setMaxVelocityScalingFactor(100.0);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setNumPlanningAttempts(10);
        move_group.setPlanningTime(5);
        move_group.setGoalTolerance(0.010);

        moveit::planning_interface::MoveGroupInterface move_group_eff(node_, "arm_end_effector");
        move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues(pos));
        move_group_eff.move();

        RCLCPP_INFO(node_->get_logger(), "finished");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
    	RCLCPP_INFO(node_->get_logger(), "requesting gripper positioning abort");
        _aborted = true;
    }
private:
    bool _aborted;
    rclcpp::Node::SharedPtr node_;
};
