#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "robot_find_game.hpp"

class RobotFindNextStepAction : public BT::SyncActionNode
{
    public:
		RobotFindNextStepAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return { BT::OutputPort<std::string>("item_name")};
        }

        virtual BT::NodeStatus tick() override
        {
        	std::string item_name;
        	double x, y;

    		RobotFindGame* game = RobotFindGame::GetRobotFindGame();
    		if (game == nullptr) {
    			std::cout << "Game pointer null" << endl;
    		} else if (game->GetNextRoundItem(item_name, x, y)) {
    			cout << "NextStep: item_name= " << item_name << endl;
    			setOutput("item_name", item_name);
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
