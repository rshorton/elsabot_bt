#pragma once

#include <string>

#include <behaviortree_cpp_v3/action_node.h>

#include "robot_find_game.hpp"

class RobotFindCheckStepAction : public BT::SyncActionNode
{
    public:
	RobotFindCheckStepAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("x"),
            	     BT::InputPort<std::string>("y") };
        }

        virtual BT::NodeStatus tick() override
        {
    		RobotFindGame* game = RobotFindGame::GetRobotFindGame();
    		if (game == nullptr) {
    			std::cout << "Game pointer null" << endl;
    		} else {
            	double x;
            	double y;

            	if (!getInput<double>("x", x)) {
            		throw BT::RuntimeError("missing x location");
            	}
            	if (!getInput<double>("y", y)) {
            		throw BT::RuntimeError("missing y location");
            	}

    			if (game->CheckRound(x, y)) {
    	            return BT::NodeStatus::SUCCESS;
    			}
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
