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

#ifndef _TRANSFORM_HELPER_HPP_
#define _TRANSFORM_HELPER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TransformHelper
{
public:
	static TransformHelper & Instance(rclcpp::Node::SharedPtr node) {
		static TransformHelper helper(node);
		return helper;
	}
	bool Transform(const std::string &frame_from, const std::string &frame_to, double &x, double &y, double &z);
	bool Transform(const std::string &frame_from, const std::string &frame_to, std::string &pos);

private:
	TransformHelper(rclcpp::Node::SharedPtr node);
	~TransformHelper() {};

private:
	rclcpp::Node::SharedPtr node_;

	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	tf2_ros::Buffer tfBuffer_;
};

#endif //_TRANSFORM_HELPER_HPP_
