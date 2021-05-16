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

void RobotSeekGame::GetPointInSameDirection(SearchPose p0, SearchPose p1,
											double x, double y, double yaw,
											SearchPose &best, double &angleDiff, double &angleTo)
{
    double a0 = atan2(p0.y - y, p0.x - x);
    double a1 = atan2(p1.y - y, p1.x - x);

    if (a0 < 0.0) {
    	a0 += 2*M_PI;
    }
    if (a1 < 0.0) {
    	a1 += 2*M_PI;
    }
    if (yaw < 0.0) {
    	yaw += 2*M_PI;
    }

    cout << "a0= " << a0 << "(" << rad_to_deg(a0)
         << "), a1= " << a1 << "(" << rad_to_deg(a1)
		 << "), yaw= " << yaw << "(" << rad_to_deg(yaw)
		 << ")" << endl;

    double diff_a0 = abs(atan2(sin(a0 - yaw), cos(a0 - yaw)));
    double diff_a1 = abs(atan2(sin(a1 - yaw), cos(a1 - yaw)));
    cout << "diff_a0= " << diff_a0 << "(" << rad_to_deg(diff_a0)
    	 << "), diff_a1= " << diff_a1 << "(" << rad_to_deg(diff_a1) << ")"
		 << endl;

    if (abs(diff_a0) < abs(diff_a1)) {
    	best = p0;
    	angleDiff = diff_a0;
    	angleTo = a0;
    } else {
    	best = p1;
    	angleDiff = diff_a1;
    	angleTo = a1;
    }
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
        	it.initial_dist = 99999.0;
//        	return false;
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

    SearchPose best;
    double angleDiff;
    double angleTo;
    GetPointInSameDirection(pose_tmp[0], pose_tmp[1], init_x, init_y, init_yaw, best, angleDiff, angleTo);
    // If nether point is mostly in same dir of robot pose, go with the closest
    if (abs(angleDiff) > 80.0*M_PI/180.0) {
    	if (pose_tmp[0].initial_dist < pose_tmp[1].initial_dist) {
    		best = pose_tmp[0];
    	} else {
    		best = pose_tmp[1];
    	}
    }

    // Find the pose in the original list (which is in the search order)
    for (size_t i = 0; i < poses_.size(); i++) {
    	if (poses_[i].x == best.x && poses_[i].y == best.y) {
    		idx_start_ = i;
    		break;
    	}
    }

    cout << "Selected initial pose x,y= " << poses_[idx_start_].x << ", " << poses_[idx_start_].y << endl;
    idx_cur_ = idx_start_;
    bFirst_ = true;

    // Determine the direction to traverse the search poses
    int next_idx = idx_cur_ + 1;
    if (next_idx >= (int)poses_.size()) {
    	next_idx = 0;
    }
    int prev_idx = idx_cur_ - 1;;
    if (prev_idx < 0) {
    	prev_idx = poses_.size() - 1;
    }

    // Pick the point before or after the start search pose in the same general
    // direction as the arrival direction to the start pose from the current robot position.
    GetPointInSameDirection(poses_[prev_idx], poses_[next_idx], poses_[idx_start_].x, poses_[idx_start_].y, angleTo, best, angleDiff, angleTo);
    cout << "Best second pose x,y= " << best.x << ", " << best.y << endl;

    // Find the pose in the original list (which is in the search order)
    dir_ = 1;
    for (size_t i = 0; i < poses_.size(); i++) {
    	if (poses_[i].x == best.x && poses_[i].y == best.y) {
    		dir_ = (int)i < idx_start_? -1: 1;
    		break;
    	}
    }
    cout << "Direction= " << dir_ << endl;
    return true;
}

bool RobotSeekGame::NextSearchPose(double &x, double &y, double &yaw, int &index)
{
	if (bFirst_) {
		bFirst_ = false;
	} else {
		idx_cur_ += dir_;
		if (idx_cur_ < 0) {
			idx_cur_ = poses_.size() - 1;
		} else if (idx_cur_ >= (int)poses_.size()) {
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
	index = idx_cur_;
	return true;
}

