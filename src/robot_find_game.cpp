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

bool RobotFindGame::Init(std::string game_data_path, std::vector<RobotFindGame::position_list_item> &positions)
{
	game_data_path_ = game_data_path;
	cout << "Robot find game data path: " << game_data_path_ << std::endl;

	idx_cur_ = -1;
	if (Load()) {
		return GetPositions(positions);
	}
	return false;
}

// Get the list of items in the order to be called for a new round
bool RobotFindGame::InitGameRound(std::string object_type, bool random)
{
	idx_cur_ = -1;
	object_type_round_ = object_type;
	vector<std::string>().swap(object_names_round_);

	if (!GetObjectTypeNames(object_type, object_names_round_)) {
		return false;
	}

	if (random) {
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		shuffle(object_names_round_.begin(), object_names_round_.end(), std::default_random_engine(seed));
	}

	std::cout << "Object list for round: ";
	for (auto it = object_names_round_.begin(); it != object_names_round_.end(); it++) {
		std::cout << *it << " ";
	}
	std::cout << endl;
	return true;
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

	int cnt = 0;
	for (auto& it: positions_) {
		cnt++;
		if (bFirst) {
			pos_first = it.second;
			bFirst = false;
		} else {
			sum_x += it.second.x;
			sum_y += it.second.y;
			double dist = (pos_first.x - it.second.x)*(pos_first.x - it.second.x) +
				   (pos_first.y - it.second.y)*(pos_first.y - it.second.y);
			cout << "position: " << it.first << ", min_dist: " << min_dist << ", dist: " << dist << std::endl;

			if (dist < min_dist) {
				min_dist = dist;
			}
		}
	}
	ave_x_ = sum_x/cnt;
	ave_y_ = sum_y/cnt;

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

	if (++idx_cur_ < (int)object_names_round_.size()) {
		// Name of object
		name = object_names_round_[idx_cur_];
		std::string pos_name = objects_[object_type_round_][name];
		position pos = positions_[pos_name];
		x = pos.x;
		y = pos.y;
		return true;
	}
	return false;
}

bool RobotFindGame::SetLocation(double x, double y)
{
	if (idx_cur_ < (int)object_names_round_.size()) {
		std::string obj_name = object_names_round_[idx_cur_];
		std::string pos_name = objects_[object_type_round_][obj_name];
		positions_[pos_name].x = x;
		positions_[pos_name].y = y;
		return Save();
	}
	return false;
}

// Consider the player at the correct location if 3 position sample received
// that are within the threshold.
bool RobotFindGame::CheckRound(double x, double y)
{
	if (idx_cur_ < (int)object_names_round_.size()) {
		std::string obj_name = object_names_round_[idx_cur_];
		std::string pos_name = objects_[object_type_round_][obj_name];
		position pos = positions_[pos_name];

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

bool RobotFindGame::GetPositions(std::vector<RobotFindGame::position_list_item> &positions)
{
	for (auto& it: positions_) {
		position_list_item p;
		p.name = it.first;
		p.pos.x = it.second.x;
		p.pos.y = it.second.y;
		positions.push_back(p);
	}
	return true;
}

bool RobotFindGame::GetObjectTypeNames(std::string object_type, std::vector<std::string> &names)
{
	if (objects_.find(object_type) == objects_.end()) {
		return false;
	}
	for (auto& it: objects_[object_type]) {
		names.push_back(it.first);
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
	positions_.clear();
	objects_.clear();

	objects_["numbers"] = std::unordered_map<std::string, std::string>();

	need_setup_ = false;

	robot_pos_x_ = game_data["robot_pos_x"].get<double>();
	robot_pos_y_ = game_data["robot_pos_y"].get<double>();
	robot_yaw_ = game_data["robot_yaw"].get<double>();

	// The positions array holds a list of objects which specify the floor locations
	// of the number cards.  It also implicitly specifies the list of number objects.
	for (auto& it: game_data["positions"]) {
		position pos;
		pos.x = it["x"].get<double>();
		pos.y = it["y"].get<double>();
		std::string name = it["name"].get<std::string>();
		positions_[name] = pos;

		objects_["numbers"][name] = name;

		if (pos.x == 0.0 && pos.y == 0.0) {
			need_setup_ = true;
		}
		cout << "name= " << name
			 << ", x= " << pos.x
			 << ", y= " << pos.y
			 << endl;
	}

	if (game_data.contains("alt_objects") && game_data["alt_objects"].is_array()) {
		cout << "Alt objects:" << endl;
		for (auto& it: game_data["alt_objects"]) {
			if (it.is_object()) {
				std::string obj_type = it["type"].get<std::string>();
				objects_[obj_type] = std::unordered_map<std::string, std::string>();
				for (auto& it2: it["objects"]) {
					std::string name = it2["name"].get<std::string>();
					std::string pos = it2["pos"].get<std::string>();
					objects_[obj_type][name] = pos;
					cout << "type: " << obj_type << ", " << name << ", at pos: " << pos << endl;
				}
			}
		}
	}
	return true;
}

// fix - retest
bool RobotFindGame::Save()
{
	// Fix - add saving of robot position at the time of game setup
	json data = json::object();
	data["positions"] = json::array();

	for (auto& it: positions_) {
		json item;
		item["name"] = it.first;
		item["x"] = it.second.x;
		item["y"] = it.second.y;
		data["positions"].push_back(item);
	}

	for (auto& it: objects_) {
		if (it.first == "numbers") {
			continue;
		}
		data["alt_objects"] = json::array();

		json alt = json::object();
		alt["type"] = it.first;
		alt["objects"] = json::array();
		for (auto it2: it.second) {
			json item;
			item["name"] = it2.first;
			item["pos"] = it2.second;
			alt["objects"].push_back(item);
		}
		data["alt_objects"].push_back(alt);
	}

	std::string data_str = data.dump();
	std::cout << "Saving game data: " << data_str << std::endl;

	std::ofstream f(game_data_path_);
	f << data_str;
	return true;
}
