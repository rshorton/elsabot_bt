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

//#include <behaviortree_cpp_v3/behavior_tree.h>

#include "robot_seek_game.hpp"

using namespace std;

static RobotSeekGame* game = nullptr;

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

inline double rad_to_deg(double rad)
{
	return rad*180.0/M_PI;
}

bool RobotSeekGame::Init(rclcpp::Node::SharedPtr node, vector<SearchPose> &poses, double init_x, double init_y, double init_yaw)
{
	idx_cur_ = 0;
	idx_start_ = 0;
	bFirst_ = 0;

	poses_ = poses;

    // Determine the first search pose
    for (auto& it : poses_) {
        nav_msgs::msg::Path path;
        double len;
        if (GetPathToPose(node, path, len, it.x, it.y)) {
        	it.initial_dist = len;
        	//temp - to view the path on rviz/web app
        	std::this_thread::sleep_for(1000ms);
        } else {
        	RCLCPP_ERROR(node->get_logger(), "Failed to get path to pose %f, %f\n", it.x, it.y);
        	return false;
        }
    }

    std::vector<SearchPose> pose_tmp = poses_;

    sort(pose_tmp.begin(), pose_tmp.end(),
        [](const SearchPose & a, const SearchPose & b) -> bool
		{
        	return a.initial_dist < b.initial_dist;
		});

    cout << "Poses sorted by dist:" << endl;
    for (auto& it : pose_tmp) {
        cout << "x= " << it.x << ", y= " << it.y << ", dist= " << it.initial_dist << endl;
    }

    double a0 = atan2(pose_tmp[0].y - init_y, pose_tmp[0].x - init_x);
    double a1 = atan2(pose_tmp[1].y - init_y, pose_tmp[1].x - init_x);

    if (a0 < 0.0) {
    	a0 += 2*M_PI;
    }
    if (a1 < 0.0) {
    	a1 += 2*M_PI;
    }
    if (init_yaw < 0.0) {
    	init_yaw += 2*M_PI;
    }

    cout << "a0= " << a0 << "(" << rad_to_deg(a0)
         << "), a1= " << a1 << "(" << rad_to_deg(a1)
		 << "), init_yaw= " << init_yaw << "(" << rad_to_deg(init_yaw)
		 << ")" << endl;

    double diff_a0 = abs(atan2(sin(a0 - init_yaw), cos(a0 - init_yaw)));
    double diff_a1 = abs(atan2(sin(a1 - init_yaw), cos(a1 - init_yaw)));
    cout << "diff_a0= " << diff_a0 << "(" << rad_to_deg(diff_a0)
    	 << "), diff_a1= " << diff_a1 << "(" << rad_to_deg(diff_a1) << ")"
		 << endl;

    const double diff_pointing_mostly_in_same_dir = 80.0*M_PI/180.0;
    int selected = -1;
    if (diff_a0 < diff_a1) {
    	if (diff_a0 < diff_pointing_mostly_in_same_dir) {
    		selected = 0;
    	}
    } else if (diff_a1 < diff_pointing_mostly_in_same_dir) {
    	selected = 1;
    }
    // If nether point is mostly in same dir of robot pose, go with the closest
    if (selected == -1) {
    	if (pose_tmp[0].initial_dist < pose_tmp[1].initial_dist) {
    		selected = 0;
    	} else {
    		selected = 1;
    	}
    }

    // Find the pose in the original list (which is in the search order)
    for (size_t i = 0; i < poses_.size(); i++) {
    	if (poses_[i].x == pose_tmp[selected].x && poses_[i].y == pose_tmp[selected].y) {
    		selected = i;
    		break;
    	}
    }

    cout << "Selected initial pose idx: " << selected << " x,y= " << poses_[selected].x << ", " << poses_[selected].y << endl;
    idx_cur_ = selected;
    idx_start_ = selected;
    bFirst_ = true;
    return true;
}

bool RobotSeekGame::NextSearchPose(double &x, double &y, double &yaw)
{
	if (bFirst_) {
		bFirst_ = false;
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
