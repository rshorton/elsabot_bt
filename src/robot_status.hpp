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

#ifndef _ROBOT_STATUS_HPP_
#define _ROBOT_STATUS_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#undef USE_ROBOT_POSE_PUBLISHER_NODE

class RobotStatus
{
public:
	static RobotStatus* Create(rclcpp::Node::SharedPtr node)
	{
		if (!robot_status_) {
			robot_status_ = new RobotStatus(node);
		}			
		return robot_status_;
	};

	static RobotStatus* GetInstance()
	{
		return robot_status_;
	};

	bool GetPose(double &x, double &y, double &z, double &yaw);

private:
	RobotStatus(rclcpp::Node::SharedPtr node);
	~RobotStatus() {};

#if defined(USE_ROBOT_POSE_PUBLISHER_NODE)
	void PoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
#endif	

private:
	static RobotStatus *robot_status_;

	rclcpp::Node::SharedPtr node_;
#if defined(USE_ROBOT_POSE_PUBLISHER_NODE)
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
#endif	
    double pos_x_;
    double pos_y_;
	double pos_z_;
    double yaw_;
    bool valid_pose_;

};

#endif //_ROBOT_STATUS_HPP_
