#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include "behaviortree_cpp/action_node.h"

#include "bt_custom_type_helpers.hpp"

class ArmGotoNamedPositionAction : public BT::ThreadedAction
{
public:
    ArmGotoNamedPositionAction(const std::string& name, const BT::NodeConfig& config)
        : BT::ThreadedAction(name, config),
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
        node_ = rclcpp::Node::make_shared("pick_object_client");

        std::string pos_in;
        if (!getInput<std::string>("position", pos_in)) {
            throw BT::RuntimeError("missing required input [position]");
        }

        RCLCPP_INFO(node_->get_logger(), "Arm position [%s]", pos_in.c_str());

        moveit::planning_interface::MoveGroupInterface move_group(node_, "xarm");
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(0.10);
        move_group.setNumPlanningAttempts(10);
        move_group.setPlanningTime(5);
        move_group.setGoalTolerance(0.010);

        move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
        move_group.move();

        RCLCPP_INFO(node_->get_logger(), "finished");
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
