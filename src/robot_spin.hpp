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

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/utils.h"

#include "geometry_msgs/msg/twist.hpp"

#include <behaviortree_cpp/action_node.h>

using namespace std;

class RobotSpin : public BT::ThreadedAction
{
    public:
		RobotSpin(const std::string& name, const BT::NodeConfig& config)
            : BT::ThreadedAction(name, config),
			  strt_yaw_(0.0),
			  prev_yaw_(0.0),
			  relative_yaw_(0.0)
        {
			std::stringstream ss;
			ss << "robot_spin_action_node" << getInstanceCnt();
			node_ = rclcpp::Node::make_shared(ss.str());
            vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<double>("angle"),
					 BT::InputPort<double>("velocity"),
					 BT::InputPort<std::string>("angular_units")
				   };
        }

        virtual BT::NodeStatus tick() override
        {
        	setStatus(BT::NodeStatus::RUNNING);
        	_aborted = false;

        	// Get the current pose
			RobotStatus* robot_status = RobotStatus::GetInstance();
			if (!robot_status) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cannot get robot status for obtaining pose");
				return BT::NodeStatus::FAILURE;
			}

			double cur_x, cur_y, cur_z, cur_yaw = 0.0;
			auto valid_pose = false;

        	for (int t = 0; t < 20; t++) {
				valid_pose = robot_status->GetPose(cur_x, cur_y, cur_z, cur_yaw);
				if (valid_pose) {
					break;
				}
        		std::this_thread::sleep_for(100ms);
        		cout << "Waiting for pose..." << endl;
        	}
        	if (!valid_pose) {
        		RCLCPP_ERROR(node_->get_logger(), "Did not receive current robot pose");
        		return BT::NodeStatus::FAILURE;
        	}

        	strt_yaw_ = cur_yaw;
        	prev_yaw_ = cur_yaw;
        	relative_yaw_ = 0.0;

			bool deg = true;
			std::string angular_units;
			if (getInput<std::string>("angular_units", angular_units) &&
				angular_units == "rad") {
				deg = false;
			}

        	delta_yaw_goal_ = 360.0;
        	getInput<double>("angle", delta_yaw_goal_);
			if (deg) {
        		delta_yaw_goal_ *= M_PI/180.0;
			}

        	vel_ = 0.1;
        	getInput<double>("velocity", vel_);
        	vel_ = copysign(vel_, delta_yaw_goal_);

        	geometry_msgs::msg::Twist cmd_vel;
        	cmd_vel.angular.z = vel_;
        	vel_pub_->publish(cmd_vel);

        	rclcpp::Rate rate(20);
        	while (rclcpp::ok()) {
        		if (robot_status->GetPose(cur_x, cur_y, cur_z, cur_yaw)) {
					
                	// From Nav2_recoveries/plugings/spin.cpp
                	double delta_yaw = cur_yaw - prev_yaw_;
                	if (abs(delta_yaw) > M_PI) {
                	    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
                	}

                	relative_yaw_ += delta_yaw;
                	prev_yaw_ = cur_yaw;

                	double remaining_yaw = abs(delta_yaw_goal_) - abs(relative_yaw_);

                	//RCLCPP_INFO(node_->get_logger(), "Spin: cur: %.2f, prev: %.2f, delta: %.2f, rel: %.2f, remain: %.2f",
                	//		cur_yaw, prev_yaw_, delta_yaw, relative_yaw_, remaining_yaw);

                	if (remaining_yaw < 1e-6 || _aborted) {
                    	cmd_vel.angular.z = 0.0;
                    	vel_pub_->publish(cmd_vel);
                    	break;
                	}

                	vel_pub_->publish(cmd_vel);
        		}
        		rclcpp::spin_some(node_);
        		rate.sleep();
        	}
        	if (_aborted) {
        		_aborted = false;
        		return BT::NodeStatus::IDLE;
        	}
       		return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override {
            _aborted = true;
        }

    private:
        static unsigned getInstanceCnt()
        {
        	static unsigned cnt = 0;
        	return ++cnt;
        }

        bool _aborted;
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

        double strt_yaw_;
        double prev_yaw_;
        double relative_yaw_;

        double delta_yaw_goal_;
        double vel_;
};

