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

#ifndef _ROS_COMMON_HPP_
#define _ROS_COMMON_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"

class ROSCommon
{
public:
	static ROSCommon* Create(rclcpp::Node::SharedPtr node)
	{
		if (!ros_common_) {
			ros_common_ = new ROSCommon(node);
		}			
		return ros_common_;
	};

	static ROSCommon* GetInstance()
	{
		return ros_common_;
	};

	rclcpp::Node::SharedPtr GetNode() {
		return node_;
	};

private:
	ROSCommon(rclcpp::Node::SharedPtr node): node_(node) {};
	~ROSCommon() {};

private:
	static ROSCommon *ros_common_;

	rclcpp::Node::SharedPtr node_;
};

#endif //_ROS_COMMON_HPP_
