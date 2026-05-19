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

#ifndef _GAME_SETTINGS_HPP_
#define _GAME_SETTINGS_HPP_

#include <string>
#include <map>
#include <json.hpp>

// Singleton pattern from https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

class GameSettings
{
    public:
        static GameSettings& getInstance(std::string path = "")
        {
            static GameSettings instance(path);
            return instance;
        }
    private:
        GameSettings(std::string path);
    public:
        GameSettings(GameSettings const&) = delete;
        void operator=(GameSettings const&) = delete;

        void Set(std::string json_settings);
        void Get(std::string &json_settings);

        bool GetStringProperty(std::string prop, std::string &val);
        bool GetNumberProperty(std::string prop, int &val);
        bool GetBoolProperty(std::string prop, bool &val);
        bool GetStringMapProperty(std::string prop, std::map<std::string, std::string> &val);

    protected:
        bool InitSettingsFilePath(std::string filename);

    protected:
        nlohmann::json settings_;
        std::string settings_path_;
};

#endif
