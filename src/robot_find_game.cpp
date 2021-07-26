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
#include <limits>
#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include <json.hpp>

#include "robot_find_game.hpp"

using json = nlohmann::json;
using namespace std;

static RobotFindGame* game = nullptr;

RobotFindGame* RobotFindGame::GetRobotFindGame()
{
	return game;
}

RobotFindGame* RobotFindGame::CreateRobotFindGame()
{
	if (game) {
		delete game;
	}
	game = new RobotFindGame;
	return game;
}

RobotFindGame::RobotFindGame():
	idx_cur_(0),
	at_pos_cnt_(0),
	need_setup_(true),
	processed_positions_(false),
	pos_threshold_(0.0),
	ave_x_(0.0),
	ave_y_(0.0),
	robot_pos_x_(0.0),
	robot_pos_y_(0.0),
	robot_yaw_(0.0)
{
}

RobotFindGame::~RobotFindGame()
{
}

bool RobotFindGame::Init(std::string game_data_path, std::vector<RobotFindGame::item> &items)
{
	game_data_path_ = game_data_path;
	cout << "Robot find game data path: " << game_data_path_ << std::endl;

	idx_cur_ = -1;
	if (Load()) {
		return GetItems(items);
	}
	return false;
}

// Get the list of items in the order to be called for a new round
bool RobotFindGame::InitGameRound(bool random)
{
	idx_cur_ = -1;
	item_names_round_ = item_names_;

	if (random) {
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		shuffle(item_names_round_.begin(), item_names_round_.end(), std::default_random_engine(seed));
	}

	std::cout << "Item list for round: ";
	for (auto it = item_names_round_.begin(); it != item_names_round_.end(); it++) {
		std::cout << *it << " ";
	}
	std::cout << endl;
	return 0;
}

// Game assumes items positioned on the floor in a grid pattern.
// Calculate the distance from the first item to all other items and
// determine the min distance.  Use 1/2 that distance as the threshold
// for determining if the current position of the player is at a
// given game position.
bool RobotFindGame::ProcessPositions()
{
	bool bFirst = true;
	position pos_first;
	double sum_x = 0;
	double sum_y = 0;
	double min_dist =std::numeric_limits<double>::max();

	for (auto it = item_names_.begin(); it != item_names_.end(); it++) {
		if (bFirst) {
			pos_first = item_map_[*it];
			bFirst = false;
		} else {
			sum_x += item_map_[*it].x;
			sum_y += item_map_[*it].y;
			double dist = (pos_first.x - item_map_[*it].x)*(pos_first.x - item_map_[*it].x) +
				   (pos_first.y - item_map_[*it].y)*(pos_first.y - item_map_[*it].y);
			cout << "item: " << *it << ", min_dist: " << min_dist << ", dist: " << dist << std::endl;

			if (dist < min_dist) {
				min_dist = dist;
			}
		}
	}
	ave_x_ = sum_x/item_names_.size();
	ave_y_ = sum_y/item_names_.size();

	min_dist = sqrt(min_dist);
	pos_threshold_ = min_dist/2.0;
	cout << "min_dist: " << min_dist << ", pos_threshold: " << pos_threshold_ << std::endl;
	return true;
}

// Get next item of the current round. Call InitGameRound first.
bool RobotFindGame::GetNextRoundItem(std::string &name, double &x, double &y)
{
	at_pos_cnt_ = 0;

	if (!processed_positions_) {
		ProcessPositions();
		processed_positions_ = true;
	}

	if (++idx_cur_ < (int)item_names_round_.size()) {
		name = item_names_round_[idx_cur_];
		position pos = item_map_[name];
		x = pos.x;
		y = pos.y;
		return true;
	}
	return false;
}

bool RobotFindGame::SetItemLocation(double x, double y)
{
	if (idx_cur_ < (int)item_names_round_.size()) {
		std::string name = item_names_round_[idx_cur_];
		position p = {x, y};
		item_map_[name] = p;
		return Save();
	}
	return false;
}

// Consider the player at the correct location if 3 position sample received
// that are within the threshold.
bool RobotFindGame::CheckRound(double x, double y)
{
	if (idx_cur_ < (int)item_names_round_.size()) {
		std::string name = item_names_round_[idx_cur_];
		position pos = item_map_[name];

		double dist = sqrt((x - pos.x)*(x - pos.x) + (y - pos.y)*(y - pos.y));
		if (dist < pos_threshold_) {
			at_pos_cnt_++;
		} else {
			at_pos_cnt_ = 0;
		}

		std::cout << "CheckRound: dist= " << dist << ", at_pos_cnt= " << at_pos_cnt_ << std::endl;
		if (at_pos_cnt_ >= 3) {
			return true;
		}
	}
	return false;
}

bool RobotFindGame::GetItems(std::vector<RobotFindGame::item> &items)
{
	for (auto it = item_names_.begin(); it != item_names_.end(); it++) {
		item i;
		i.name = *it;
		i.pos.x = item_map_[*it].x;
		i.pos.y = item_map_[*it].y;
		items.push_back(i);
	}
	return true;
}

bool RobotFindGame::Load()
{
	json game_data;
	std::ifstream f(game_data_path_);
	if(f.fail()) {
		return false;
	}
	f >> game_data;

	// Game data will contain x,y = 0,0 initially.  The x,y position for each item needs to be
	// specified by setting-up the game.
	item_names_.clear();
	item_map_.clear();

	need_setup_ = false;

	robot_pos_x_ = game_data["robot_pos_x"].get<double>();
	robot_pos_y_ = game_data["robot_pos_y"].get<double>();
	robot_yaw_ = game_data["robot_yaw"].get<double>();

	for (json::iterator it = game_data["items"].begin(); it != game_data["items"].end(); ++it) {
		position pos;
		pos.x = (*it)["x"].get<double>();
		pos.y = (*it)["y"].get<double>();
		std::string name = (*it)["name"].get<std::string>();
		item_map_[name] = pos;
		item_names_.push_back(name);

		if (pos.x == 0.0 && pos.y == 0.0) {
			need_setup_ = true;
		}

		cout << "name= " << name
			 << ", x= " << pos.x
			 << ", y= " << pos.y
			 << endl;
	}
	return true;
}

bool RobotFindGame::Save()
{
	// Fix - add saving of robot position at the time of game setup
	json data = json::object();
	data["items"] = json::array();

	for (auto it = item_names_.begin(); it != item_names_.end(); it++) {
		json item;
		item["name"] = *it;
		item["x"] = item_map_[*it].x;
		item["y"] = item_map_[*it].y;
		data["items"].push_back(item);
	}
	std::string data_str = data.dump();
	std::cout << "Saving game data: " << data_str << std::endl;

	std::ofstream f(game_data_path_);
	f << data_str;
	return true;
}
