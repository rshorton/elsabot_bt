#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "robot_says_game.hpp"

using namespace std;

class RobotSaysInitAction : public BT::SyncActionNode
{
    public:
		RobotSaysInitAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("level_start"), BT::InputPort<std::string>("level_end") };
        }

        virtual BT::NodeStatus tick() override
        {
        	RobotSaysGame* game = RobotSaysGame::GetRobotSaysGame();
			if (game == nullptr) {
				game = RobotSaysGame::CreateRobotSaysGame();
			}
        	int32_t level_start = game->GetMinDifficulty();
        	int32_t level_end = game->GetMaxDifficulty();
        	getInput<std::int32_t>("level_start", level_start);
        	getInput<std::int32_t>("level_end", level_end);

        	game->Init(level_start, level_end);
//        	game->TestGameData();
            return BT::NodeStatus::SUCCESS;
        }

    private:
};

