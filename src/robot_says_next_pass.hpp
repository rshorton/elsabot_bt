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

#include "robot_says_game.hpp"

using namespace std;

class RobotSaysNextPassAction : public BT::SyncActionNode
{
    public:
		RobotSaysNextPassAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
        	return{ BT::OutputPort<std::string>("level_desc") };
        }

        virtual BT::NodeStatus tick() override
        {
        	std::string level_desc;

        	RobotSaysGame* game = RobotSaysGame::GetRobotSaysGame();
			if (game != nullptr &&
				!game->NextPass(false, level_desc)) {
				setOutput("level_desc", level_desc);
	            return BT::NodeStatus::SUCCESS;
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
