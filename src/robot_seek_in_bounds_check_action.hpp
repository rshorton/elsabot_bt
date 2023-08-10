/*
Copyright 2023 Scott Horton

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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "robot_seek_game.hpp"
#include "bt_custom_type_helpers.hpp"
#include "transform_helper.hpp"

using namespace std;

class RobotSeekInBoundsCheckAction : public BT::SyncActionNode
{
    public:
		RobotSeekInBoundsCheckAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("position") };
        }

        virtual BT::NodeStatus tick() override
        {
        	RobotSeekGame* game = RobotSeekGame::GetRobotSeekGame();
			if (game == nullptr) {
    			throw BT::RuntimeError("Game pointer null");
    		}

			std::string position_str;					
			if (!getInput<std::string>("position", position_str)) {
				throw BT::RuntimeError("missing position");
			}

			auto pos = BT::convertFromString<Position>(position_str);

			auto helper = TransformHelper::GetInstance();
			if (!helper) {
				return BT::NodeStatus::FAILURE;
			}

			double z = 0;
			if (helper->Transform("oakd_center_camera", "map", pos.x, pos.y, z)) {
				if (game->InBoundary(pos)) {
					return BT::NodeStatus::SUCCESS;
				}
			}				
			return BT::NodeStatus::FAILURE;
        }
};
