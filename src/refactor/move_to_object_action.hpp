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

#include <string>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <behaviortree_cpp/action_node.h>

#include "bt_custom_type_helpers.hpp"
#include "ros_common.hpp"

using namespace std;

class MoveToObjectAction : public BT::ThreadedAction
{
    public:
		MoveToObjectAction(const std::string& name, const BT::NodeConfig& config) :
            BT::ThreadedAction(name, config)
        {
			node_ = ROSCommon::GetInstance()->GetNode();
            vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("pose"),
					 BT::InputPort<double>("velocity"),
					 BT::InputPort<double>("target_dist")
				   };
        }

        virtual BT::NodeStatus tick() override
        {
        	setStatus(BT::NodeStatus::RUNNING);
        	_aborted = false;

        	vel_ = 0.1;
        	getInput<double>("velocity", vel_);

        	target_dist_ = 0.1;
        	getInput<double>("target_dist", target_dist_);

        	geometry_msgs::msg::Twist cmd_vel;
        	cmd_vel.angular.z = vel_;
        	vel_pub_->publish(cmd_vel);

			double last_rot_vel = std::numeric_limits<double>::infinity();

        	while (!_aborted && rclcpp::ok()) {

				// Since this code runs in a separate thread from the main thread, and
				// since the detector_processor code isn't currently thread safe,
				// another tree node reads the current obj postion and provides to this
				// node
				std::string pose_str;					
				if (!getInput<std::string>("pose", pose_str)) {
					throw BT::RuntimeError("missing pose");
				}

// fix - getting a position now instead of x,y,yaw
		        Pose2D obj = BT::convertFromString<Pose2D>(pose_str);

				double dist = sqrt(obj.x*obj.x + obj.y*obj.y);

				RCLCPP_INFO(node_->get_logger(), "relative obj pos: x,y: %f, %f, dist: %f", obj.x, obj.y, dist);	

				dist -= target_dist_;
				if (dist < 0) {
					dist = 0.0;
				}

				double xVel = 0.0;
				if (dist > 0.08) {
					xVel = 0.05;
				} else if (dist > 0.01) {
					xVel = 0.03;
				}

				double angle = 0.0;
				if (abs(obj.y) > 0.0 || abs(obj.x) > 0.0) {
					angle = atan2(obj.y, obj.x);
				}

				double rot_vel = 0.0;
				if (abs(angle) > M_PI/8) {
					rot_vel = vel_;
					// If turned away too much, then focus on
					// getting turned toward object before moving
					// to it.  This avoids moving forward and the object
					// falling out of view.
					xVel = 0.01;
				} else if (abs(angle) > M_PI/32) {
					rot_vel = vel_/2;
				}
	        	rot_vel = copysign(rot_vel, angle);


				bool atPos = false;
				if (xVel == 0.0 && 
					(rot_vel == 0.0 ||
					(last_rot_vel != std::numeric_limits<double>::infinity() && signbit(rot_vel) != signbit(last_rot_vel)))) {
					atPos = true;
					rot_vel = 0.0;
				}

				RCLCPP_INFO(node_->get_logger(), "angle: %f, rot_vel: %f, last_rot_vel: %f, xVel: %f, atPos: %d",
					angle, rot_vel, last_rot_vel, xVel, atPos);

				last_rot_vel = rot_vel;

        		geometry_msgs::msg::Twist cmd_vel;
        		cmd_vel.angular.z = rot_vel;
				cmd_vel.linear.x = xVel;
        		vel_pub_->publish(cmd_vel);

				if (atPos) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
        	}
        	if (_aborted) {
        		_aborted = false;
        		return BT::NodeStatus::IDLE;
        	}
       		return BT::NodeStatus::SUCCESS;
        }

		void stop()
		{
       		geometry_msgs::msg::Twist cmd_vel;
      		cmd_vel.angular.z = 0.0;
			cmd_vel.linear.x = 0.0;
       		vel_pub_->publish(cmd_vel);
		}

        virtual void halt() override {
			stop();
            _aborted = true;
			RCLCPP_INFO(node_->get_logger(), "MoveToRelativePositionAction aborted");
        }

    private:
        bool _aborted;
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

        double vel_;
		double target_dist_;
};

