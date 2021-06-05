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
	    bool spin;
	    bool scan;

	    double initial_dist;
	};

	RobotSeekGame();
	~RobotSeekGame();

	static RobotSeekGame* GetRobotSeekGame();
	static RobotSeekGame* CreateRobotSeekGame();

	bool Init(rclcpp::Node::SharedPtr node, std::string datapath, double init_x, double init_y, double init_yaw, std::vector<SearchPose> &poses);
	bool NextSearchPose(double &x, double &y, double &yaw, bool &spin_at_goal, bool &scan_at_goal, int &index);

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
