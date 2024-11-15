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
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"

#include <behaviortree_cpp/action_node.h>

#include "robot_seek_game.hpp"

using namespace std;

class RobotSeekInitAction : public BT::SyncActionNode
{
    public:
		RobotSeekInitAction(const std::string& name, const BT::NodeConfig& config)
            : BT::SyncActionNode(name, config),
			  cur_pos_x(0.0),
			  cur_pos_y(0.0),
			  cur_yaw(0.0)
        {
			node_ = rclcpp::Node::make_shared("robot_seek_init_action_node");
            search_path_pub_ = node_->create_publisher<nav_msgs::msg::Path> ("robot_seek_game/search_path", 1);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("game_file") };
        }

        virtual BT::NodeStatus tick() override
        {
            RobotStatus* robot_status = RobotStatus::GetInstance();
            if (!robot_status) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cannot get robot status for obtaining pose");
                return BT::NodeStatus::FAILURE;
            }
            
            double x, y, z, yaw;
            auto got_pose = robot_status->GetPose(x, y, z, yaw);
            if (!got_pose) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Robot pose not available.");
                return BT::NodeStatus::FAILURE;
            }

        	RobotSeekGame* game = RobotSeekGame::GetRobotSeekGame();
			if (game == nullptr) {
				game = RobotSeekGame::CreateRobotSeekGame();
			}

			const char* dir = std::getenv("GAME_DATA_DIR");
			if (!dir) {
				throw BT::RuntimeError("Please env var GAME_DATA_DIR to the directory holding the game data/settings files.");
			}

        	std::string game_file;
        	if (!getInput<std::string>("game_file", game_file)) {
        		throw BT::RuntimeError("missing game file mame");
        	}

        	std::stringstream ss;
        	ss << dir << "/" << game_file;
        	cout << "Game filepath: " << ss.str();

        	std::vector<RobotSeekGame::SearchPose> poses;
        	if (game->Init(node_, ss.str(), cur_pos_x, cur_pos_y, cur_yaw, poses)) {

                std::vector<geometry_msgs::msg::PoseStamped> path_poses;
                cout << "Parsed poses:" << endl;
                for (auto& it : poses) {
                	geometry_msgs::msg::PoseStamped msg;
                	msg.header.frame_id = "map";
                	msg.header.stamp = rclcpp::Time();
                	msg.pose.position.x = it.x;
                	msg.pose.position.y = it.y;
                	msg.pose.position.z = 0;
                	msg.pose.orientation = orientationAroundZAxis(it.yaw);
                	path_poses.push_back(msg);

                    cout << "x= " << it.x
                    	 << ", y= " << it.y
    					 << ", yaw= " << it.yaw
    					 << ", spin= " << it.spin
    					 << ", scan= " << it.scan
    					 << endl;
                }

                nav_msgs::msg::Path path;
                path.header.frame_id = "map";
                path.header.stamp = rclcpp::Time();
                path.poses = path_poses;
                search_path_pub_->publish(path);

#if 0
        		// test
        		double x, y, yaw;
        		int index;
        		while(game->NextSearchPose(x, y, yaw, index));
#endif
        		return BT::NodeStatus::SUCCESS;
        	} else {
        		return BT::NodeStatus::FAILURE;
        	}
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr search_path_pub_;
        double cur_pos_x;
        double cur_pos_y;
        double cur_yaw;
};

