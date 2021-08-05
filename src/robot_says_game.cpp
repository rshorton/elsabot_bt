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

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include "robot_says_game.hpp"

using namespace std;

static RobotSaysGame* game = nullptr;

RobotSaysGame* RobotSaysGame::GetRobotSaysGame()
{
	return game;
}

RobotSaysGame* RobotSaysGame::CreateRobotSaysGame()
{
	if (game) {
		delete game;
	}
	game = new RobotSaysGame;
	return game;
}

RobotSaysGame::RobotSaysGame():
	easy_(true),
	level_start_(0),
	level_end_(0),
	step_index_(0)
{
	speech_[OnHip] = "put your * hand on your hip";
	speech_[ArmOut] = "stretch your * arm out";
	speech_[ArmToSide] = "put your * arm to your side";
	speech_[TShoulder] = "touch your shoulder with your * hand";
	speech_[TStomach] = "put your * hand on your stomach";
	speech_[AHead] = "put your * hand high above your head";
	speech_[OHead] = "put your * hand on your head";
	//speech_[TNeck] = "put your * hand on your neck";

	name_[OnHip] = "OnHip";
	name_[ArmOut] = "ArmOut";
	name_[ArmToSide] = "ArmToSide";
	name_[TShoulder] = "TouchingShoulder";
	name_[TStomach] = "TouchingStomach";
	name_[AHead] = "Abovehead";
	name_[OHead] = "OnHead";
	//name_[TNeck] = "TouchingNeck";

	poses_easy_ = {ArmOut, ArmToSide, AHead};
	poses_all_ = {OnHip, ArmOut, ArmToSide, TShoulder, TStomach, AHead, OHead /*, TNeck*/};

	levels_ = {SideAny, Left, Right, LeftAndRight};
	level_desc_ = {"Either left or right", "Left hand only", "Right hand only", "Left and Right hands"};

	Init(easy_, level_start_, level_end_);
}

RobotSaysGame::~RobotSaysGame()
{
}

int RobotSaysGame::NextPass(bool reset, std::string &level_desc)
{
	if (reset) {
		level_cur_ = level_start_;
	} else if (level_cur_ < level_end_) {
		level_cur_++;
	} else {
		return -1;
	}

	if (easy_) {
		random_poses_ = poses_easy_;
	} else {
		random_poses_ = poses_all_;
	}
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	shuffle(random_poses_.begin(), random_poses_.end(), std::default_random_engine(seed));

	std::cout << "Random pose list: ";
	for (auto it = random_poses_.begin(); it != random_poses_.end(); it++) {
		std::cout << name_[*it] << " ";
	}
	std::cout << endl;
	level_desc = level_desc_[level_cur_];
	step_index_ = 0;
	return 0;
}

int RobotSaysGame::NextStep(string &pose_name_l, string &pose_name_r, string &pose_lr_check, string &pose_speech, int32_t &step_index)
{
	pose_name_l = "";
	pose_name_r = "";
	pose_speech = "";

	if (random_poses_.size() == 0) {
		return -1;
	}

	std::cout << "NextStep: Remaining in pose list: ";
	for (auto it = random_poses_.begin(); it != random_poses_.end(); it++) {
		std::cout << name_[*it] << " ";
	}
	std::cout << endl;

	Pose p = random_poses_.back();
	random_poses_.pop_back();
	pose_speech = speech_[p];
	step_index = step_index_++;

	size_t marker =  pose_speech.find ('*');
	string side;
	if (marker != string::npos) {
		switch (levels_[level_cur_]) {
		default:
		case SideAny:
			side = "";
			pose_name_l = name_[p];
			pose_name_r = name_[p];
			pose_lr_check = "any";
			break;
		case Left:
			side = "left";
			pose_name_l = name_[p];
			pose_lr_check = "left";
			break;
		case Right:
			side = "right";
			pose_name_r = name_[p];
			pose_lr_check = "right";
			break;
		case LeftAndRight:
			if ((rand() % 2) == 0) {
				pose_name_l = name_[p];
				side = "left";
			} else {
				pose_name_r = name_[p];
				side = "right";
			}
			pose_lr_check = side;
			break;
		}
		pose_speech.replace(marker, 1, side);
	}
	std::cout << "NextStep: pose_name_l= " << pose_name_l
			  << ", pose_name_r= " << pose_name_r
			  << ", pose_speech= " << pose_speech
			  << ", step_index= " << step_index
			  << endl;
	return 0;
}

void RobotSaysGame::Init(bool easy, int32_t level_start, int32_t level_end)
{
	easy_ = easy;

	if (level_end > GetMaxDifficulty()) {
		level_end = GetMaxDifficulty();
	}
	if (level_start > level_end) {
		level_start = 0;
	}
	level_start_ = level_start;
	level_end_ = level_end;

	std::string level_desc;
	NextPass(true, level_desc);
}

void RobotSaysGame::DumpSteps()
{
	string pose_name_l;
	string pose_name_r;
	string pose_lr_check;
	string pose_speech;
	int32_t step_index;

	while (!NextStep(pose_name_l, pose_name_r, pose_lr_check, pose_speech, step_index)) {
		std::cout << "NextStep: pose_name_l= " << pose_name_l << ", pose_name_r= " << pose_name_r << ", pose_lr_check= " << pose_lr_check << ", pose_speech= " << pose_speech << endl;
	}
	std::cout << endl << endl;

}

void RobotSaysGame::TestGameData()
{
	std::string level_desc;
	NextPass(true, level_desc);
	DumpSteps();
	NextPass(false, level_desc);
	DumpSteps();
	NextPass(false, level_desc);
	DumpSteps();
	NextPass(false, level_desc);
	DumpSteps();
}

int32_t RobotSaysGame::GetMinDifficulty()
{
	return 0;
}

int32_t RobotSaysGame::GetMaxDifficulty()
{
	return levels_.size() - 1;
}
