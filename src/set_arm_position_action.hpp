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
#include "ros_common.hpp"

#define TO_RAD(x) ((x)/180.0*M_PI)

#undef FUTURE_WAIT_BLOCK

class SetArmPositionAction : public BT::ThreadedAction
{
public:
    SetArmPositionAction(const std::string& name, const BT::NodeConfig& config)
        : BT::ThreadedAction(name, config),
		 _aborted(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        
        return{ BT::InputPort<std::string>("position"),
                BT::InputPort<std::string>("orientation")
              };
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = ROSCommon::GetInstance()->GetNode();

        std::string pos_in;
        if (!getInput<std::string>("position", pos_in)) {
            throw BT::RuntimeError("missing required input [position]");
        }

        Position pos = BT::convertFromString<Position>(pos_in);

        std::string orient_in;
        if (!getInput<std::string>("orientation", orient_in)) {
            throw BT::RuntimeError("missing required input [orientation]");
        }

        OrientationRPY orient = BT::convertFromString<OrientationRPY>(orient_in);

        RCLCPP_INFO(node_->get_logger(), "Arm goal, position %f %f %f, orient: %f %f %f",
            pos.x, pos.y, pos.z, orient.r, orient.p, orient.y);

        if (pos.z < -0.120) {
            pos.z = -0.120;
            RCLCPP_INFO(node_->get_logger(), "Limiting object z to %f", pos.z);
        }

        moveit::planning_interface::MoveGroupInterface move_group(node_, "xarm");
        move_group.setMaxVelocityScalingFactor(2.0);
        move_group.setMaxAccelerationScalingFactor(0.10);
        move_group.setNumPlanningAttempts(10);
        move_group.setPlanningTime(5);
        move_group.setGoalTolerance(0.010);

        tf2::Quaternion q;
        geometry_msgs::msg::Pose target;

        // Move to above pick position
        q.setRPY(TO_RAD(orient.r), TO_RAD(orient.p), TO_RAD(orient.y));
        target.orientation = tf2::toMsg(q);

        target.position.x = pos.x;
        target.position.y = pos.y;
        target.position.z = pos.z;

        RCLCPP_INFO(node_->get_logger(), "Moving arm");
        move_group.setPoseTarget(target);
        move_group.move();

        RCLCPP_INFO(node_->get_logger(), "finished");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
    	RCLCPP_INFO(node_->get_logger(), "Requesting set arm position abort");
        _aborted = true;
    }
private:
    bool _aborted;
    rclcpp::Node::SharedPtr node_;
};
