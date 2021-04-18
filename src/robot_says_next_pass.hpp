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
