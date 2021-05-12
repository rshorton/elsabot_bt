#ifndef _ROBOT_SEEK_GAME_HPP_
#define _ROBOT_SEEK_GAME_HPP_

#include <vector>
#include <string>
#include <map>

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

	bool Init(rclcpp::Node::SharedPtr node, std::vector<SearchPose> &poses, double init_x, double init_y, double init_yaw);
	bool NextSearchPose(double &x, double &y, double &yaw, int &index);

private:
	void GetPointInSameDirection(SearchPose p0, SearchPose p1, double x, double y, double yaw,
								 SearchPose &best, double &angleDiff, double &angleTo);

private:
	std::vector<SearchPose> poses_;
	int idx_cur_;
	int idx_start_;
	int dir_;
	bool bFirst_;
};

#endif
