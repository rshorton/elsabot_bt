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

#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cstdlib>
#include <ctime>

#include "robot_cat_game.hpp"
#include "bt_custom_type_helpers.hpp"

namespace {

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

}

std::shared_ptr<RobotCatGame> RobotCatGame::game_ = nullptr;

std::shared_ptr<RobotCatGame> RobotCatGame::GetRobotCatGame()
{
	if (!game_) {
		game_ = std::make_shared<RobotCatGame>();
	}
	return game_;
}

RobotCatGame::RobotCatGame()
{
}

bool RobotCatGame::Init(double timeout_sec, double yaw_min, double yaw_max, double yaw_step, double yaw_evade_step,
						double pitch_min, double pitch_max, double pitch_step, double pitch_dither_step)
{
	if (set_value(timeout_sec_, timeout_sec)) {
		timeout_sec_ *= 1000.0;
	}

	set_value(yaw_min_, yaw_min);
	set_value(yaw_max_, yaw_max);
	set_value(yaw_step_, yaw_step);
	set_value(yaw_evade_step_, yaw_evade_step);

	set_value(pitch_min_, pitch_min);
	set_value(pitch_max_, pitch_max);
	set_value(pitch_step_, pitch_step);

	set_value(pitch_dither_step_, pitch_dither_step);

	return true;
}

bool RobotCatGame::NextHeadPose(const robot_head_interfaces::msg::TrackStatus &ts, OrientationRPY &orient, std::string &action_desc)
{
	RCLCPP_DEBUG(rclcpp::get_logger("elsabot_bt"), "NextHeadPos: track status, tracking= %d, moving= %d, pan= %f, " \
				 "tilt= %f, rot= %f, bb_x_min= %f, bb_y_min= %f, bb_x_max= %f, bb_y_max= %f, conf= %f, yar_dir= %d, move_to_limit= %d",
				 ts.tracking, ts.moving, ts.pan_angle, ts.tilt_angle,
				 ts.rotate_angle, ts.object.bb_x_min, ts.object.bb_y_min, ts.object.bb_x_max,
				 ts.object.bb_y_max, ts.object.confidence, yaw_dir_, move_to_limit_);

	action_desc = "none";

	// Wait until previous move finished
	if (!ts.moving) {
		orient.r = ts.rotate_angle;
		orient.p = ts.tilt_angle;
		orient.y = ts.pan_angle;

		bool move = false;

		// Move if bounding box of cat too close to center of view where the laser
		// is expected to shine.

		auto yaw_step = yaw_step_;

		if (ts.object.bb_x_max > 0.4 && ts.object.bb_x_min < 0.6) {
			move = true;
			move_to_limit_ = false;
			RCLCPP_INFO(rclcpp::get_logger("elsabot_bt"), "NextHeadPos: cat is close, move now");
			action_desc = "moving";
			yaw_step = yaw_evade_step_;
		}

		if (move || move_to_limit_) {
			orient.y += yaw_step*yaw_dir_;
			if (orient.y > yaw_max_ ||
				orient.y < yaw_min_) {
				yaw_dir_ *= -1;

				RCLCPP_INFO(rclcpp::get_logger("elsabot_bt"), "NextHeadPos: at yaw limit, reverse yaw dir");
				move_to_limit_ = false;
			}

			// Zig-zag the pitch as the head moves in yaw direction.
			orient.p += pitch_step_*pitch_dir_;
			if (orient.p > pitch_max_ ||
				orient.p < pitch_min_) {
				pitch_dir_ *= -1;

				RCLCPP_INFO(rclcpp::get_logger("elsabot_bt"), "NextHeadPos: at pitch limit, reverse pitch dir");
			}

			time_last_move_ = std::chrono::steady_clock::now();

		} else {
			// If cat not close enough, then after N seconds, sweep the head to the farther limit.
			std::chrono::duration<double, std::milli> since_last_move = std::chrono::steady_clock::now() - time_last_move_;

			if (since_last_move.count() > timeout_sec_) {
				action_desc = "timeout";
				move_to_limit_ = true;
				// Go in direction farthest from limit
				yaw_dir_ = sgn(orient.y*-1);
				RCLCPP_INFO(rclcpp::get_logger("elsabot_bt"), "NextHeadPos: timeout, start move to limit, dir= %d", yaw_dir_);
			} else {
				action_desc = "waiting";
			}

			std::chrono::duration<double, std::milli> since_last_small_step = std::chrono::steady_clock::now() - time_last_small_step_;
			if (since_last_small_step.count() > 500) {
				// Move the head slowly in the pitch direction while waiting for the cat to move.
				orient.p += pitch_dither_step_*pitch_dither_dir_;
				if (orient.p > pitch_max_ ||
					orient.p < pitch_min_) {
					pitch_dither_dir_ *= -1;
				}
				time_last_small_step_ = std::chrono::steady_clock::now();
			}				
		}

		if (orient.y > yaw_max_) {
			orient.y = yaw_max_;
		} else if (orient.y < yaw_min_) {
			orient.y = yaw_min_;
		}

		if (orient.p > pitch_max_) {
			orient.p = pitch_max_;
		} else if (orient.p < pitch_min_) {
			orient.p = pitch_min_;
		}
		return true;
	}
	return false;
}

bool RobotCatGame::set_value(double &param, double value)
{
	if (value != UNSET()) {
		param = value;
		return true;
	}
	return false;
}	

