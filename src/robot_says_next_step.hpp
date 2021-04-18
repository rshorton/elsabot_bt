#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "robot_says_game.hpp"

using namespace std;

class RobotSaysNextStepAction : public BT::SyncActionNode
{
    public:
		RobotSaysNextStepAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<std::string>("pose_name_l"), BT::OutputPort<std::string>("pose_name_r"), BT::OutputPort<std::string>("pose_lr_check"), BT::OutputPort<std::string>("pose_speech")};
        }

        virtual BT::NodeStatus tick() override
        {
    		string pose_name_l;
    		string pose_name_r;
    		string pose_lr_check;
    		string pose_speech;

    		RobotSaysGame* game = RobotSaysGame::GetRobotSaysGame();
    		if (game == nullptr) {
    			std::cout << "Game pointer null" << endl;
    		}

    		if (game != nullptr && !game->NextStep(pose_name_l, pose_name_r, pose_lr_check, pose_speech)) {
    			cout << "NextStep: pose_name_l= " << pose_name_l << ", pose_name_r= " << pose_name_r << ", pose_lr_check= " << pose_lr_check << ", pose_speech= " << pose_speech << endl;
    			setOutput("pose_name_l", pose_name_l);
    			setOutput("pose_name_r", pose_name_r);
    			setOutput("pose_lr_check", pose_lr_check);
    			setOutput("pose_speech", pose_speech);
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
