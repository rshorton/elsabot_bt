#ifndef _ROBOT_SEEK_GAME_HPP_
#define _ROBOT_SEEK_GAME_HPP_

#include <vector>
#include <string>
#include <map>

using namespace std;

class RobotSeekGame
{
private:

public:
	struct SearchPose
	{
	    double x;
	    double y;
	    double yaw;
	    double initial_dist;
	};

	RobotSeekGame();
	~RobotSeekGame();

	static RobotSeekGame* GetRobotSeekGame();
	static RobotSeekGame* CreateRobotSeekGame();

	bool Init(rclcpp::Node::SharedPtr node, std::string search_poses, double init_x, double init_y, double init_yaw);
	bool NextSearchPose(double &x, double &y, double &yaw);

//	void DumpSteps();
//	void TestGameData();


private:
	vector<SearchPose> poses_;
	unsigned int idx_cur_;
	unsigned int idx_start_;
	bool bFirst_;
};

#endif
