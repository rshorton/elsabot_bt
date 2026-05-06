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

#pragma once

#ifndef _ROBOT_STATUS_HPP_
#define _ROBOT_STATUS_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "robot_head_interfaces/msg/track_status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class RobotStatus
{
public:
	static RobotStatus* Create(rclcpp::Node::SharedPtr node)
	{
		if (!robot_status_) {
			robot_status_ = new RobotStatus(node);
		}			
		return robot_status_;
	}

	static RobotStatus* GetInstance()
	{
		return robot_status_;
	}

	bool GetPose(double &x, double &y, double &z, double &yaw) const;
	bool GetTrackStatus(robot_head_interfaces::msg::TrackStatus &status) const;
	bool GetBatteryStatus(sensor_msgs::msg::BatteryState &status) const;

private:
	RobotStatus(rclcpp::Node::SharedPtr node);
	~RobotStatus() {};

	void TrackStatusCallback(robot_head_interfaces::msg::TrackStatus::SharedPtr msg);
	void BatteryStatusCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

private:
	static RobotStatus *robot_status_;

	rclcpp::Node::SharedPtr node_;

	bool valid_track_status_ = false;
    rclcpp::Subscription<robot_head_interfaces::msg::TrackStatus>::SharedPtr track_status_sub_;
    robot_head_interfaces::msg::TrackStatus track_status_;

	bool valid_battery_status_ = false;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_status_sub_;
	sensor_msgs::msg::BatteryState battery_status_;
};

#endif //_ROBOT_STATUS_HPP_
