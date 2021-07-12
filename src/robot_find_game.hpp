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
	} item;

	RobotFindGame();
	~RobotFindGame();

	static RobotFindGame* GetRobotFindGame();
	static RobotFindGame* CreateRobotFindGame();

	bool Init(std::string game_data_file, std::vector<RobotFindGame::item> &items);
	bool NeedsSetup() { return need_setup_; }

	bool SetItemLocation(double x, double y);
	bool GetNextItem(std::string cur_item, std::string &next_item);

	bool InitGameRound(bool random);
	bool CheckRound(double x, double y);

	bool GetNextRoundItem(std::string &name, double &x, double &y);

	bool GetItems(std::vector<RobotFindGame::item> &items);

	void GetAvePosition(position &ave_pos) { ave_pos.x = ave_x_; ave_pos.y = ave_y_; }
	void GetRobotPosition(double &x, double &y, double &yaw) { x = robot_pos_x_; y = robot_pos_y_; yaw = robot_yaw_; }

protected:
	bool Load();
	bool Save();

	bool ProcessPositions();

private:
	std::string game_data_path_;
	std::map<std::string, position> item_map_;
	std::vector<std::string> item_names_;
	std::vector<std::string> item_names_round_;

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
