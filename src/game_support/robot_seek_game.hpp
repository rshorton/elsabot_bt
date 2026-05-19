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

#ifndef _ROBOT_SEEK_GAME_HPP_
#define _ROBOT_SEEK_GAME_HPP_

#include <vector>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "bt_custom_type_helpers.hpp"


class RobotSeekGame
{
private:

public:
	struct SearchPose
	{
	    double x;
	    double y;
	    double yaw;
	    bool spin;
	    bool scan;

	    double initial_dist;
	};

	RobotSeekGame();
	~RobotSeekGame();

	static RobotSeekGame* GetRobotSeekGame();
	static RobotSeekGame* CreateRobotSeekGame();

	bool Init(rclcpp::Node::SharedPtr node, std::string datapath, double init_x, double init_y, double init_yaw, std::vector<SearchPose> &poses);
	bool NextSearchPose(double &x, double &y, double &yaw, bool &spin_at_goal, bool &scan_at_goal, int &index);

	bool InBoundary(const Position &pos) const;

private:
	void GetPointInSameDirection(SearchPose p0, SearchPose p1, double x, double y, double yaw,
								 SearchPose &best, double &angleDiff, double &angleTo);

	void TestBoundaryCheck() const;

private:
	rclcpp::Node::SharedPtr node_;
	std::vector<Position> boundary_;
	std::vector<SearchPose> poses_;
	int idx_cur_;
	int idx_start_;
	int dir_;
	bool bFirst_;
};

#endif
