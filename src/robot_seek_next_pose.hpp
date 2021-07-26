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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "robot_seek_game.hpp"

using namespace std;

class RobotSeekNextSearchPose : public BT::SyncActionNode
{
    public:
		RobotSeekNextSearchPose(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<std::string>("next_goal"),
            	    BT::OutputPort<std::string>("goal_speech"),
					BT::OutputPort<bool>("spin_at_goal"),
					BT::OutputPort<bool>("scan_at_goal") };
        }

        virtual BT::NodeStatus tick() override
        {
    		string goal;
    		string goal_speech;

        	RobotSeekGame* game = RobotSeekGame::GetRobotSeekGame();
			if (game == nullptr) {
    			throw BT::RuntimeError("Game pointer null");
    		}

			double x, y, yaw;
			bool spin_at_goal = false;
			bool scan_at_goal = false;
			int index;
    		if (game->NextSearchPose(x, y, yaw, spin_at_goal, scan_at_goal, index)) {
    			std::stringstream ss;

    			ss << x << ',' << y << ',' << yaw;
    			cout << "NextSearchPose: x,y,yaw= " << ss.str() << " index= " << index << endl;
    			setOutput("next_goal", ss.str());

    			ss.str("");

    			ss << "At search point " << (index + 1);
    			setOutput("goal_speech", ss.str());
    			setOutput("spin_at_goal", spin_at_goal);
    			setOutput("scan_at_goal", scan_at_goal);
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
