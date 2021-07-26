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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "robot_find_game.hpp"

class RobotFindSetPositionAction : public BT::SyncActionNode
{
    public:
	RobotFindSetPositionAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("x"),
            		 BT::InputPort<std::string>("y")};
        }

        virtual BT::NodeStatus tick() override
        {
        	double x, y;
        	if (!getInput<double>("x", x)) {
        		throw BT::RuntimeError("missing x position");
        	}
        	if (!getInput<double>("y", y)) {
        		throw BT::RuntimeError("missing y position");
        	}

    		RobotFindGame* game = RobotFindGame::GetRobotFindGame();
    		if (game == nullptr) {
    			std::cout << "Game pointer null" << endl;
    		} else if (game->SetItemLocation(x, y)) {
    			cout << "Current item pos (x,y): " << x << ", " << y << endl;
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
