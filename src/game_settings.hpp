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
