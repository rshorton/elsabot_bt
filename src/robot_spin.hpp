#pragma once

#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

//#include "std_msgs/msg/header.hpp"
//#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/quaternion.hpp"
//#include "tf2/transform_datatypes.h"
//#include "tf2/LinearMath/Quaternion.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/twist.hpp"

#include <behaviortree_cpp_v3/action_node.h>

using namespace std;

class RobotSpin : public BT::AsyncActionNode
{
    public:
		RobotSpin(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config),
			  cur_pos_x(0.0),
			  cur_pos_y(0.0),
			  valid_pose(false),
			  strt_yaw_(0.0),
			  cur_yaw_(0.0),
			  prev_yaw_(0.0),
			  relative_yaw_(0.0)
        {
			node_ = rclcpp::Node::make_shared("robot_spin_action_node");

            pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
              "/robot_pose",
              rclcpp::SystemDefaultsQoS(),
              std::bind(&RobotSpin::poseCallback, this, std::placeholders::_1));

            vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<double>("angle"), BT::InputPort<double>("velocity") };
        }

        void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
        	cur_pos_x = msg->pose.position.x;
        	cur_pos_y = msg->pose.position.y;
        	cur_yaw_ = tf2::getYaw(msg->pose.orientation);
        	valid_pose = true;
        	//cout << "Received pose, yaw= " << cur_yaw_ << ", x= " << cur_pos_x << ", y= " << cur_pos_y << endl;
        }

        virtual BT::NodeStatus tick() override
        {
        	setStatus(BT::NodeStatus::RUNNING);
        	_aborted = false;

        	for (int t = 0; t < 20; t++) {
            	rclcpp::spin_some(node_);

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

        	valid_pose = false;
        	strt_yaw_ = cur_yaw_;
        	prev_yaw_ = cur_yaw_;
        	relative_yaw_ = 0.0;

        	delta_yaw_goal_ = 360.0;
        	getInput<double>("angle", delta_yaw_goal_);
        	delta_yaw_goal_ *= M_PI/180.0;

        	vel_ = 0.1;
        	getInput<double>("velocity", vel_);
        	vel_ = copysign(vel_, delta_yaw_goal_);

        	geometry_msgs::msg::Twist cmd_vel;
        	cmd_vel.angular.z = vel_;
        	vel_pub_->publish(cmd_vel);

        	rclcpp::Rate rate(20);
        	while (rclcpp::ok()) {
        		if (valid_pose) {
                	// From Nav2_recoveries/plugings/spin.cpp
                	double delta_yaw = cur_yaw_ - prev_yaw_;
                	if (abs(delta_yaw) > M_PI) {
                	    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
                	}

                	relative_yaw_ += delta_yaw;
                	prev_yaw_ = cur_yaw_;

                	double remaining_yaw = abs(delta_yaw_goal_) - abs(relative_yaw_);

                	//RCLCPP_INFO(node_->get_logger(), "Spin: cur: %.2f, prev: %.2f, delta: %.2f, rel: %.2f, remain: %.2f",
                	//		cur_yaw_, prev_yaw_, delta_yaw, relative_yaw_, remaining_yaw);

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
        bool _aborted;
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

        double cur_pos_x;
        double cur_pos_y;
        bool valid_pose;

        double strt_yaw_;
        double cur_yaw_;
        double prev_yaw_;
        double relative_yaw_;

        double delta_yaw_goal_;
        double vel_;
};

