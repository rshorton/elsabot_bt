/*
Copyright 2023 Scott Horton

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

#ifndef _ROBOT_CAT_GAME_HPP_
#define _ROBOT_CAT_GAME_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <float.h>


#include "rclcpp/rclcpp.hpp"
#include "robot_head_interfaces/msg/track_status.hpp"
#include "bt_custom_type_helpers.hpp"

class RobotCatGame
{
public:
	RobotCatGame();

	static std::shared_ptr<RobotCatGame> GetRobotCatGame();

	bool Init(double timeout_sec, double yaw_min, double yax_max, double yaw_step, double yaw_evade_step,
		double pitch_min, double pitch_max, double pitch_step, double pitch_dither_step);

	bool NextHeadPose(const robot_head_interfaces::msg::TrackStatus &track_status, OrientationRPY &orient, std::string &action_desc);

	static double UNSET()
	{
		return DBL_MAX;
	};

private:
	bool set_value(double &param, double value);

private:
	static std::shared_ptr<RobotCatGame> game_;
	double yaw_min_ = -50.0;		// min/max movement in yaw direction (degrees)
	double yaw_max_ = 50.0;
	double yaw_step_ = 15.0;
	double yaw_evade_step_ = 30.0;

	double pitch_min_ = -45.0;		// min/max movement in pitch direction (degrees)	
	double pitch_max_ = -30.0;
	double pitch_step_ = 2.0;

	double pitch_dither_step_ = 1.0;

	double timeout_sec_ = 10000.0;

	int yaw_dir_ = 1;
	int pitch_dir_ = 1;
	int pitch_dither_dir_ = 1;
	bool move_to_limit_ = true;
	std::chrono::time_point<std::chrono::steady_clock> time_last_move_;	
	std::chrono::time_point<std::chrono::steady_clock> time_last_small_step_;
};

#endif
