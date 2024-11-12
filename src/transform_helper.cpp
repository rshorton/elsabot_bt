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

#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp/action_node.h"

#include "transform_helper.hpp"

const std::string DEFAULT_SOURCE_FRAME = "oakd";

TransformHelper *TransformHelper::xform_helper_ = nullptr;

TransformHelper::TransformHelper(rclcpp::Node::SharedPtr node)
	: node_(node),
  	  tfBuffer_(node->get_clock())
{
	tfl_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
}

bool TransformHelper::GetTransform(const std::string &frame_from, const std::string &frame_to, geometry_msgs::msg::TransformStamped &transform)
{
	try{
		transform = tfBuffer_.lookupTransform(frame_to, frame_from, rclcpp::Time(0.1));
		return true;
	} catch (tf2::TransformException &ex) {
		RCLCPP_ERROR(node_->get_logger(), "TransformHelper: Failed to transform from %s to %s",
			frame_from.c_str(), frame_to.c_str());
	}
	return false;
}

bool TransformHelper::Transform(const std::string &frame_from, const std::string &frame_to, double &x, double &y, double &z)
{
	try{
		geometry_msgs::msg::TransformStamped transformStamped;
		transformStamped = tfBuffer_.lookupTransform(frame_to, frame_from, rclcpp::Time(0.1));

		geometry_msgs::msg::PointStamped pt;
		pt.point.x = x;
		pt.point.y = y;
		pt.point.z = z;
		pt.header.stamp = node_->now();
		pt.header.frame_id = frame_from;

		geometry_msgs::msg::PointStamped transformed_pt;
		tf2::doTransform(pt, transformed_pt, transformStamped);

		RCLCPP_DEBUG(node_->get_logger(), "TransformHelper: in (%s) x,y,z: (%f, %f, %f) => out (%s) x,y,z: (%f, %f, %f)",
										frame_from.c_str(), x, y, z,
										frame_to.c_str(), transformed_pt.point.x, transformed_pt.point.y, transformed_pt.point.z);
		x = transformed_pt.point.x;
		y = transformed_pt.point.y;
		z = transformed_pt.point.z;

	} catch (tf2::TransformException &ex) {
		RCLCPP_ERROR(node_->get_logger(), "TransformHelper: Failed to transform from %s to %s",
			frame_from.c_str(), frame_to.c_str());
			return false;
	}
	return true;
}

bool TransformHelper::Transform(const std::string &frame_from, const std::string &frame_to, std::string &pos)
{
	// Input position is comma separated xyz 
    auto parts = BT::splitString(pos, ',');
    if (parts.size() != 3) {
        throw BT::RuntimeError("invalid position input)");
    }

    double x = BT::convertFromString<double>(parts[0]);
    double y = BT::convertFromString<double>(parts[1]);
	double z = BT::convertFromString<double>(parts[2]);

	if (Transform(frame_from == "" || frame_from == "camera"? DEFAULT_SOURCE_FRAME: frame_from,
		frame_to, x, y, z)) {
		std::stringstream ss;
		ss << x << "," << y << "," << z;
		pos = ss.str();
	} else {
		pos = "";
		return false;
	}		
	return true;
}	

