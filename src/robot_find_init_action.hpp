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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"

#include <behaviortree_cpp/action_node.h>

#include "robot_find_game.hpp"

class RobotFindInitAction : public BT::SyncActionNode
{
    public:
		RobotFindInitAction(const std::string& name, const BT::NodeConfig& config)
            : BT::SyncActionNode(name, config)
        {
			node_ = rclcpp::Node::make_shared("robot_find_init_action_node");

			// fix - use a more generic topic for publishing game info on the map
            path_pub_ = node_->create_publisher<nav_msgs::msg::Path> ("robot_seek_game/search_path", 1);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("game_file"),
        		     BT::InputPort<std::string>("object_type"),
        		     BT::InputPort<bool>("random_order"),
                     BT::OutputPort<std::string>("needs_init"),
					 BT::OutputPort<std::string>("robot_pose")};
					 //BT::OutputPort<double>("angle_to_field")};
        }

        virtual BT::NodeStatus tick() override
        {
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

        	RobotFindGame* game = RobotFindGame::GetRobotFindGame();
			if (game == nullptr) {
				game = RobotFindGame::CreateRobotFindGame();
			}

			const char* dir = std::getenv("GAME_DATA_DIR");
			if (!dir) {
				throw BT::RuntimeError("Please set env var GAME_DATA_DIR to the directory holding the game data/settings files.");
			}

        	std::string game_file;
        	std::string object_type;
        	bool random_order = false;

        	if (!getInput<std::string>("game_file", game_file)) {
        		throw BT::RuntimeError("missing robot find game file name");
        	}

        	if (!getInput<std::string>("object_type", object_type)) {
        		throw BT::RuntimeError("missing robot find game object type");
        	}

        	getInput<bool>("random_order", random_order);

        	std::stringstream ss;
        	ss << dir << "/" << game_file;
        	cout << "Game filepath: " << ss.str() << std::endl;

        	// Init the game from the game data file
        	std::vector<RobotFindGame::position_list_item> positions;
        	if (game->Init(ss.str(), positions)) {
        		setOutput("needs_init", game->NeedsSetup()? "1": "0");
        		if (!game->InitGameRound(object_type, random_order)) {
        			throw BT::RuntimeError("invalid object type, not supported in find game json");
        		}

        		// Publish the list of item locations for display via the webui.
        		// Fix - use a more general message that allows the label to be
        		// specified.  The path poses message happens to work here since
        		// the webui displays a number for each location.
                std::vector<geometry_msgs::msg::PoseStamped> path_poses;
                cout << "Parsed positions:" << endl;
                for (auto& it : positions) {
                	geometry_msgs::msg::PoseStamped msg;
                	msg.header.frame_id = "map";
                	msg.header.stamp = rclcpp::Time();
                	msg.pose.position.x = it.pos.x;
                	msg.pose.position.y = it.pos.y;
                	msg.pose.position.z = 0;
                	path_poses.push_back(msg);

                    cout << "x= " << it.pos.x
                    	 << ", y= " << it.pos.y
    					 << ", name= " << it.name
    					 << endl;
                }

                nav_msgs::msg::Path path;
                path.header.frame_id = "map";
                path.header.stamp = rclcpp::Time();
                path.poses = path_poses;
                path_pub_->publish(path);

                // Return the robot position for the game
                double x, y, yaw;
                game->GetRobotPosition(x, y, yaw);
                std::stringstream ss;
                ss << x << "," << y << "," << yaw;
                cout << "Robot pose: " << ss.str() << std::endl;
                setOutput("robot_pose", ss.str());

#if 0
                // Calc how much to rotate robot so that it faces the field of play.
                // Another BT node will use this delta to rotate the robot.
                RobotFindGame::position ave;
                game->GetAvePosition(ave);
                double angle = atan2(ave.y - cur_pos_y, ave.x - cur_pos_x);
                angle *= 180.0/M_PI;
                double yaw = cur_yaw_*180.0/M_PI;
                double delta = angle - yaw;
                cout << "Angle: " << angle << ", yaw: " << yaw << ", delta: " << delta << std::endl;
                setOutput("angle_to_field", delta);
#endif
        		return BT::NodeStatus::SUCCESS;
        	} else {
        		return BT::NodeStatus::FAILURE;
        	}
        }

    private:
        rclcpp::Node::SharedPtr node_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        double cur_pos_x_;
};

