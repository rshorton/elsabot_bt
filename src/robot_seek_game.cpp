#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nav2_compute_path_client_util.hpp"

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "robot_seek_game.hpp"

using namespace std;

static RobotSeekGame* game = nullptr;

namespace BT
{
template <> inline
RobotSeekGame::SearchPose convertFromString(StringView key)
{
    // three real numbers (x,y) separated by comma
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 3) {
        throw BT::RuntimeError("invalid input)");
    } else {
    	RobotSeekGame::SearchPose pose;
    	pose.x = convertFromString<double>(parts[0]);
    	pose.y = convertFromString<double>(parts[1]);
    	pose.yaw = convertFromString<double>(parts[2]);
    	pose.initial_dist = 0.0;
        return pose;
    }
}
}

RobotSeekGame* RobotSeekGame::GetRobotSeekGame()
{
	return game;
}

RobotSeekGame* RobotSeekGame::CreateRobotSeekGame()
{
	if (game) {
		delete game;
	}
	game = new RobotSeekGame;
	return game;
}

RobotSeekGame::RobotSeekGame():
	idx_cur_(0),
	idx_start_(0),
	bFirst_(false)
{
}

RobotSeekGame::~RobotSeekGame()
{
}

bool RobotSeekGame::Init(rclcpp::Node::SharedPtr node, std::string search_poses, double init_x, double init_y, double init_yaw)
{
	idx_cur_ = 0;
	idx_start_ = 0;
	bFirst_ = 0;

	// Parse the ordered search pose list
    auto poses_in = BT::splitString(search_poses, ';');
    if (poses_in.size() == 0) {
        throw BT::RuntimeError("invalid input)");
    } else {
    	for (unsigned i = 0; i < poses_in.size(); i++) {
    		poses_.push_back(BT::convertFromString<RobotSeekGame::SearchPose>(poses_in[i]));
    	}
    }
    if (poses_.size() == 0) {
        throw BT::RuntimeError("invalid input)");
    }
    cout << "Parsed poses:" << endl;
    for (auto& it : poses_) {
        cout << "x= " << it.x << ", y= " << it.y << ", yaw= " << it.yaw << endl;
    }

    // Determine the first search pose
    for (auto& it : poses_) {
        nav_msgs::msg::Path path;
        double len;
        if (GetPathToPose(node, path, len, it.x, it.y)) {
        	it.initial_dist = len;
        } else {
        	RCLCPP_ERROR(node->get_logger(), "Failed to get path to pose %f, %f\n", it.x, it.y);
        	return false;
        }
    }

    sort(poses_.begin(), poses_.end(),
        [](const SearchPose & a, const SearchPose & b) -> bool
		{
        	return a.initial_dist > b.initial_dist;
		});

    cout << "Poses sorted by dist:" << endl;
    for (auto& it : poses_) {
        cout << "x= " << it.x << ", y= " << it.y << ", dist= " << it.initial_dist << endl;
    }

    double a0 = atan2(poses_[0].y - init_y, poses_[0].x - init_x);
    double a1 = atan2(poses_[1].y - init_y, poses_[1].x - init_x);

    cout << "a0= " << a0 << ", a1= " << a1 << ", init_yaw= " << init_yaw << endl;

    return true;

}

bool RobotSeekGame::NextSearchPose(double &x, double &y, double &yaw)
{
	if (bFirst_) {
		bFirst_ = false;
		idx_cur_ = idx_start_;
	} else {
		if (++idx_cur_ >= poses_.size()) {
			idx_cur_ = 0;
		}
		if (idx_cur_ == idx_start_) {
			return false;
		}
	}

	x = poses_[idx_cur_].x;
	y = poses_[idx_cur_].y;
	yaw = poses_[idx_cur_].yaw;

	cout << "NextPose: x= " << x << ", y= " << y << ", yaw= " << yaw << endl;
	return true;
}

#if 0
void RobotSeekGame::TestGameData()
{
	NextPose(true, level_desc);
	NextPass(false, level_desc);
	NextPass(false, level_desc);
	NextPass(false, level_desc);
}
#endif
