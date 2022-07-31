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

/*
This class is used to create and contain Object Detection Processor
instances.  A tree node can request the creation and other nodes can
query the status of the detector at any time. 
*/

#include <iostream>
#include <string>
#include <map>
#include <chrono>

#include "imu_topic.hpp"

IMUTopic *IMUTopic::imu_topic_ = nullptr;

IMUTopic::IMUTopic(rclcpp::Node::SharedPtr node):
	node_(node)
{
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
		"/imu/data",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&IMUTopic::IMUCallback, this, std::placeholders::_1));
}

void IMUTopic::IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg)
{
	RCLCPP_DEBUG(node_->get_logger(), "IMU msg received");
	last_msg_ = msg;
}

bool IMUTopic::GetAngularVelocity(double &x, double &y, double &z)
{
	if (last_msg_) {
		x = last_msg_->angular_velocity.x;
		y = last_msg_->angular_velocity.y;
		z = last_msg_->angular_velocity.z;
		return true;
	}
	return false;
}

