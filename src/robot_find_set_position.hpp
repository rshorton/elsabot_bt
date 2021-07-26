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
    		} else if (game->SetLocation(x, y)) {
    			cout << "Current item pos (x,y): " << x << ", " << y << endl;
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
