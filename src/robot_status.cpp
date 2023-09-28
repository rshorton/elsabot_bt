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
#include "transform_helper.hpp"

RobotStatus *RobotStatus::robot_status_ = nullptr;

RobotStatus::RobotStatus(rclcpp::Node::SharedPtr node):
	node_(node)
{
	track_status_sub_ = node_->create_subscription<robot_head_interfaces::msg::TrackStatus>(
				"/head/track_status",
				rclcpp::SystemDefaultsQoS(),
				std::bind(&RobotStatus::TrackStatusCallback, this, std::placeholders::_1));
	RCLCPP_DEBUG(node_->get_logger(), "Sub to /head/track_status");
}

void RobotStatus::TrackStatusCallback(robot_head_interfaces::msg::TrackStatus::SharedPtr msg)
{
	track_status_ = *msg;
	valid_track_status_ = true;
	RCLCPP_DEBUG(node_->get_logger(), "Got track status");
}

bool RobotStatus::GetPose(double &x, double &y, double &z, double &yaw)
{
	geometry_msgs::msg::TransformStamped xf;
	if (!TransformHelper::GetInstance()->GetTransform("map", "base_link", xf)) {
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

	RCLCPP_DEBUG(node_->get_logger(), "Robot pose: x,y,yaw: %f, %f, %f",
		pos_x_, pos_y_, yaw_);
	return true;		
}

bool RobotStatus::GetTrackStatus(robot_head_interfaces::msg::TrackStatus &status)
{
	if (valid_track_status_) {
		status = track_status_;
		return true;
	}
	return false;
}

	