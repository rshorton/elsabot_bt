/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <iostream>
#include <string>
#include <map>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "robot_status.hpp"

RobotStatus *RobotStatus::robot_status_ = nullptr;

RobotStatus::RobotStatus(rclcpp::Node::SharedPtr node):
	node_(node),
	transform_helper_(TransformHelper::Instance(node_)),
	pos_x_(0.0), 
	pos_y_(0.0),
	pos_z_(0.0),
	yaw_(0.0),
	valid_pose_(false)
{
#if defined(USE_ROBOT_POSE_PUBLISHER_NODE)	
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/robot_pose",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RobotStatus::PoseCallback, this, std::placeholders::_1));
#endif		
}

#if defined(USE_ROBOT_POSE_PUBLISHER_NODE)	
void RobotStatus::PoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	pos_x_ = msg->pose.position.x;
	pos_y_ = msg->pose.position.y;
	pos_z_ = msg->pose.position.z;

	double quatx = msg->pose.orientation.x;
	double quaty = msg->pose.orientation.y;
	double quatz = msg->pose.orientation.z;
	double quatw = msg->pose.orientation.w;

	tf2::Quaternion q(quatx, quaty, quatz, quatw);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw_ = yaw;

	valid_pose_ = true;
	RCLCPP_DEBUG(node_->get_logger(), "Robot pose: x,y,yaw: %f, %f, %f",
		pos_x_, pos_y_, yaw_);
}

bool RobotStatus::GetPose(double &x, double &y, double &z, double &yaw)
{
	if (valid_pose_) {
		x = pos_x_;
		y = pos_y_;
		z = pos_z_;
		yaw = yaw_;
	} else {
		x = 0.0;
		y = 0.0;
		z = 0.0;
		yaw = 0.0;
	}
	return valid_pose_;
}

#else

bool RobotStatus::GetPose(double &x, double &y, double &z, double &yaw)
{
	geometry_msgs::msg::TransformStamped xf;
	if (!transform_helper_.GetTransform("map", "base_link", xf)) {
		x = 0.0;
		y = 0.0;
		z = 0.0;
		yaw = 0.0;
		return false;
	}

	pos_x_ = xf.transform.translation.x;
	pos_y_ = xf.transform.translation.y;
	pos_z_ = xf.transform.translation.z;

	double quatx = xf.transform.rotation.x;
	double quaty = xf.transform.rotation.y;
	double quatz = xf.transform.rotation.z;
	double quatw = xf.transform.rotation.w;

	tf2::Quaternion q(quatx, quaty, quatz, quatw);
	tf2::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw);
	yaw_ = yaw;

	valid_pose_ = true;
	RCLCPP_INFO(node_->get_logger(), "Robot pose: x,y,yaw: %f, %f, %f",
		pos_x_, pos_y_, yaw_);

	valid_pose_ = true;
	return valid_pose_;		
}
#endif
	