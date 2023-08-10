/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include <json.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nav2_compute_path_client_util.hpp"

#include "robot_seek_game.hpp"

using json = nlohmann::json;
using namespace std;

#undef USE_POSE_LIST_ORDER_FOR_DEMO

namespace ns {
	struct SearchPose;
}

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

bool RobotSeekGame::Init(rclcpp::Node::SharedPtr node, std::string datapath,
						 double init_x, double init_y, double init_yaw, vector<RobotSeekGame::SearchPose> &poses)
{
	node_ = node;
	idx_cur_ = 0;
	idx_start_ = 0;
	bFirst_ = 0;

	json game_data;
	std::ifstream f(datapath);
	f >> game_data;

	poses_.clear();

	// Read the poses where the robot will search
	cout << "Game search poses: " << endl;
	for (json::iterator it = game_data["search_poses"].begin(); it != game_data["search_poses"].end(); ++it) {
		SearchPose sp;
		sp.x = (*it)["x"].get<double>();
		sp.y = (*it)["y"].get<double>();
		sp.yaw = (*it)["yaw"].get<double>();
		sp.spin = (*it)["spin"].get<bool>();
		sp.scan = (*it)["scan"].get<bool>();

		poses_.push_back(sp);
		cout << "x= " << sp.x
		     << ", y= " << sp.y
			 << ", yaw= " << sp.yaw
			 << ", spin= " << sp.spin
			 << ", scan= " << sp.scan
			 << endl;
	}

	// Read the game boundary
	boundary_.clear();
	cout << "Game boundary points: " << endl;
	for (json::iterator it = game_data["boundary"].begin(); it != game_data["boundary"].end(); ++it) {
		Position pos;
		pos.x = (*it)["x"].get<double>();
		pos.y = (*it)["y"].get<double>();
		pos.z = 0.0;

		boundary_.push_back(pos);
		cout << "x= " << pos.x
		     << ", y= " << pos.y
			 << endl;
	}

	TestBoundaryCheck();


    // Determine the first search pose
    for (auto& it : poses_) {
        nav_msgs::msg::Path path;
        double len;
        if (GetPathToPose(node, path, len, it.x, it.y)) {
        	it.initial_dist = len;
        	//temp - to view the path on rviz/web app
        	//std::this_thread::sleep_for(300ms);
        } else {
        	RCLCPP_ERROR(node->get_logger(), "Failed to get path to pose %f, %f\n", it.x, it.y);
        	it.initial_dist = 99999.0;
        }
    }

#ifdef USE_POSE_LIST_ORDER_FOR_DEMO
    dir_ = 1;
    bFirst_ = true;
#else
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
#endif
    poses = poses_;
    return true;
}

bool RobotSeekGame::NextSearchPose(double &x, double &y, double &yaw, bool &spin_at_goal, bool &scan_at_goal, int &index)
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
	spin_at_goal = poses_[idx_cur_].spin;
	scan_at_goal = poses_[idx_cur_].scan;

	cout << "NextPose: x= " << x << ", y= " << y << ", yaw= " << yaw << endl;
	index = idx_cur_;
	return true;
}

bool RobotSeekGame::InBoundary(const Position &pos) const
{
	// Consider in the boundary if no boundary specified
    int vertices = boundary_.size();
    if (vertices < 3) {
		return true;
	}
    bool inside = false;
    for (int i = 0, j = vertices - 1; i < vertices; j = i++) {
        if (((boundary_[i].y > pos.y) != (boundary_[j].y > pos.y)) &&
            (pos.x < (boundary_[j].x - boundary_[i].x) * (pos.y - boundary_[i].y) / 
			(boundary_[j].y - boundary_[i].y) + boundary_[i].x)) {
            inside = !inside;
        }
    }
   	RCLCPP_INFO(node_->get_logger(), "InBoundary, ck: (%f, %f), inside: %d", pos.x, pos.y, inside);

    return inside;
}

void RobotSeekGame::TestBoundaryCheck() const
{
	Position pnts[] = {
		Position( -0.038488, -2.554656, 0.0),
		Position( -0.024584, -2.552068, 0.0),
		Position(  0.007693, -2.542701, 0.0),
		Position(  0.105245, -2.468326, 0.0),
		Position( -0.835761, -0.622372, 0.0),
		Position(  1.93,     -1.7,		0.0),
		Position(  2.0,      -4.0,      0.0),
		Position(  1.7,      -6.0,      0.0),
		Position(  1.5,		 -8.0,      0.0),
		Position(  0.04,	 -9.8,		0.0)};

	std::vector<Position>  positions(std::begin(pnts), std::end(pnts));
	for (const auto &pos: positions) {
		InBoundary(pos);
	}
}
