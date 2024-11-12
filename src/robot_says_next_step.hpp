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
#include <behaviortree_cpp/action_node.h>

#include "robot_says_game.hpp"

using namespace std;

class RobotSaysNextStepAction : public BT::SyncActionNode
{
    public:
		RobotSaysNextStepAction(const std::string& name, const BT::NodeConfig& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<std::string>("pose_name_l"),
            	    BT::OutputPort<std::string>("pose_name_r"),
					BT::OutputPort<std::string>("pose_lr_check"),
					BT::OutputPort<std::string>("pose_speech"),
					BT::OutputPort<int>("step_index") };
        }

        virtual BT::NodeStatus tick() override
        {
    		string pose_name_l;
    		string pose_name_r;
    		string pose_lr_check;
    		string pose_speech;
    		int32_t step_index;

    		RobotSaysGame* game = RobotSaysGame::GetRobotSaysGame();
    		if (game == nullptr) {
    			std::cout << "Game pointer null" << endl;
    		}

    		if (game != nullptr && !game->NextStep(pose_name_l, pose_name_r, pose_lr_check, pose_speech, step_index)) {
    			cout << "NextStep: pose_name_l= " << pose_name_l << ", pose_name_r= " << pose_name_r << ", pose_lr_check= " << pose_lr_check << ", pose_speech= " << pose_speech << endl;
    			setOutput("pose_name_l", pose_name_l);
    			setOutput("pose_name_r", pose_name_r);
    			setOutput("pose_lr_check", pose_lr_check);
    			setOutput("pose_speech", pose_speech);
    			setOutput("step_index", step_index);
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
