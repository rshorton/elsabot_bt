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

#ifndef _ROBOT_FIND_GAME_HPP_
#define _ROBOT_FIND_GAME_HPP_

#include <vector>
#include <string>
#include <map>

class RobotFindGame
{
private:

public:
	typedef struct
	{
	    double x;
	    double y;
	} position;

	typedef struct
	{
		position pos;
		std::string name;
	} position_list_item;

	RobotFindGame();
	~RobotFindGame();

	static RobotFindGame* GetRobotFindGame();
	static RobotFindGame* CreateRobotFindGame();

	bool Init(std::string game_data_file, std::vector<RobotFindGame::position_list_item> &positions);
	bool NeedsSetup() { return need_setup_; }

	bool SetLocation(double x, double y);
	bool GetNextItem(std::string cur_item, std::string &next_item);

	bool InitGameRound(std::string object_type, bool random);
	bool CheckRound(double x, double y);

	bool GetNextRoundItem(std::string &name, double &x, double &y);

	bool GetPositions(std::vector<RobotFindGame::position_list_item> &positions);

	bool GetObjectTypeNames(std::string object_type, std::vector<std::string> &names);

	void GetAvePosition(position &ave_pos) { ave_pos.x = ave_x_; ave_pos.y = ave_y_; }
	void GetRobotPosition(double &x, double &y, double &yaw) { x = robot_pos_x_; y = robot_pos_y_; yaw = robot_yaw_; }

protected:
	bool Load();
	bool Save();

	bool ProcessPositions();

private:
	std::string game_data_path_;

	// Map of floor positions ("1", "2", "3"..."9") to room x,y location
	std::unordered_map<std::string, position> positions_;

	// Map of object_type ("numbers", "shapes") to map of specific object vs. floor position name
	// objects_["numbers"] contains a map where the key is the name of the number, the value is the name of the position
	//    map["1"] = "1"
	//    ...
	//    map["9"] = "9"
	// objects_["shapes"] contains a map where the key is the name of the shape, the value is the name of the position
	//    map["heart"] = "1"
	//    ...
	//    map["triangle"] = "9"
	std::unordered_map<std::string, std::unordered_map<std::string, std::string>> objects_;

	std::vector<std::string> object_names_round_;
	std::string object_type_round_;

	int idx_cur_;
	int at_pos_cnt_;
	bool need_setup_;
	bool processed_positions_;
	double pos_threshold_;
	double ave_x_;
	double ave_y_;
	double robot_pos_x_;
	double robot_pos_y_;
	double robot_yaw_;
};

#endif
