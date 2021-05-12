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
            return{ BT::OutputPort<std::string>("next_goal"), BT::OutputPort<std::string>("goal_speech")};
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
			int index;
    		if (game->NextSearchPose(x, y, yaw, index)) {
    			std::stringstream ss;

    			ss << x << ',' << y << ',' << yaw;
    			cout << "NextSearchPose: x,y,yaw= " << ss.str() << " index= " << index << endl;
    			setOutput("next_goal", ss.str());

    			ss.str("");

    			ss << "At search point " << (index + 1);
    			setOutput("goal_speech", ss.str());
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
