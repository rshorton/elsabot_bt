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

#include <string>
#include <iostream>
#include <fstream>
#include <ostream>
#include <sstream>
#include <json.hpp>

#include "game_settings.hpp"

using namespace std;
using json = nlohmann::json;


GameSettings::GameSettings(std::string filename)
{
	if (filename.length()) {
		if (InitSettingsFilePath(filename)) {
			std::ifstream f(settings_path_);
			f >> settings_;
		}
	}
}

bool GameSettings::InitSettingsFilePath(std::string filename)
{
	const char* dir = std::getenv("GAME_DATA_DIR");
	if (!dir) {
		cout << "Cannot initialize setting file path. Please env var GAME_DATA_DIR to the directory holding the game data/settings files." << std::endl;
		return false;
	}
	std::stringstream ss;
	ss << dir << "/" << filename;
	settings_path_ = ss.str();
	cout << "Game settings filepath: " << settings_path_ << std::endl;
	return true;
}

void GameSettings::Set(std::string json_settings)
{
	cout << "Set settings: " << json_settings << std::endl;
	if (json_settings.length() > 0) {
		settings_ = json::parse(json_settings);
		// Save to file
		if (settings_path_.length() > 0) {
			std::ofstream f(settings_path_);
			f << settings_.dump();
		}
	}
}

void GameSettings::Get(std::string &json_settings)
{
	json_settings = settings_.dump();
}

bool GameSettings::GetStringProperty(std::string prop, std::string &val)
{
	if (settings_.contains(prop) &&
		settings_[prop].is_string()) {
		val = settings_[prop];
		return true;
	}
	return false;
}

bool GameSettings::GetNumberProperty(std::string prop, int &val)
{
	if (settings_.contains(prop) &&
		settings_[prop].is_number()) {
		val = settings_[prop];
		return true;
	}
	return false;
}

bool GameSettings::GetBoolProperty(std::string prop, bool &val)
{
	if (settings_.contains(prop) &&
		settings_[prop].is_boolean()) {
		val = settings_[prop];
		return true;
	}
	return false;
}

bool GameSettings::GetStringMapProperty(std::string prop, std::map<std::string, std::string> &val)
{
	if (settings_.contains(prop) &&
		settings_[prop].is_object()) {
		// doesn't work
		//val = settings_[prop];
		const json& rh = settings_[prop];
        for (auto& element : rh.items()) {
        	val[element.key()] = element.value();
        }
		return true;
	}
	return false;
}
