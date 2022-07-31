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

#ifndef _IMU_TOPIC_HPP_
#define _IMU_TOPIC_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUTopic
{
public:
	static IMUTopic* Create(rclcpp::Node::SharedPtr node)
	{
		if (!imu_topic_) {
			imu_topic_ = new IMUTopic(node);
		}			
		return imu_topic_;
	};

	static IMUTopic* GetInstance()
	{
		return imu_topic_;
	};

	bool GetAngularVelocity(double &x, double &y, double &z);

private:
	IMUTopic(rclcpp::Node::SharedPtr node);
	~IMUTopic() {};

	void IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg);

private:
	static IMUTopic *imu_topic_;

	rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	
	sensor_msgs::msg::Imu::SharedPtr last_msg_;
};

#endif //_IMU_TOPICS_HPP_
