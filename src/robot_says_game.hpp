#ifndef _ROBOT_SAYS_GAME_HPP_
#define _ROBOT_SAYS_GAME_HPP_

#include <vector>
#include <string>
#include <map>

using namespace std;

class RobotSaysGame
{
private:
	enum Pose {
		OnHip,
		ArmOut,
		ArmToSide,
		TShoulder,	// touching
		TStomach,	// touching
		AHead,		// above
		OHead,		// on
		TNeck,		// touching
	};

	enum Sides {
		SideAny = 0,	// Don't specify left or right, allow either
		Left,			// Specify left only
		Right,			// Specify right only
		LeftAndRight	// Specify left or right randomly
	};

public:
	RobotSaysGame();
	~RobotSaysGame();

	static RobotSaysGame* GetRobotSaysGame();
	static RobotSaysGame* CreateRobotSaysGame();

	int NextPass(bool reset);

	// Type: any, l, r, lr
	int NextStep(string &pose_name_l, string &pose_name_r, string &pose_lr_check, string &pose_speech);
	void Init(int32_t level_start, int32_t level_end);
	void DumpSteps();
	void TestGameData();

	int32_t GetMinDifficulty();
	int32_t GetMaxDifficulty();

private:
	vector<enum Pose> poses_;
	vector<enum Pose> random_poses_;
	map<enum Pose, string> speech_;
	map<enum Pose, string> name_;
	vector<enum Sides> levels_;
	int32_t level_start_;
	int32_t level_end_;
	int32_t level_cur_;
};

#endif
