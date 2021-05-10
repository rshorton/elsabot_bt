#pragma once

#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/path.hpp"

#include <behaviortree_cpp_v3/action_node.h>

#include "robot_seek_game.hpp"

using namespace std;

namespace BT
{
template <> inline
RobotSeekGame::SearchPose convertFromString(StringView key)
{
    // three real numbers (x,y) separated by comma
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 3) {
        throw BT::RuntimeError("invalid input)");
    } else {
    	RobotSeekGame::SearchPose pose;
    	pose.x = convertFromString<double>(parts[0]);
    	pose.y = convertFromString<double>(parts[1]);
    	pose.yaw = convertFromString<double>(parts[2]);
    	pose.initial_dist = 0.0;
        return pose;
    }
}
}

class RobotSeekInitAction : public BT::SyncActionNode
{
    public:
		RobotSeekInitAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config),
			  cur_pos_x(0.0),
			  cur_pos_y(0.0),
			  cur_yaw(0.0),
			  valid_pose(false)
        {
			node_ = rclcpp::Node::make_shared("robot_seek_init_action_node");
            pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
              "/robot_pose",
              rclcpp::SystemDefaultsQoS(),
              std::bind(&RobotSeekInitAction::poseCallback, this, std::placeholders::_1));

            search_path_pub_ = node_->create_publisher<nav_msgs::msg::Path> ("robot_seek_game/search_path", 1);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("search_poses") };
        }

        void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
        	cur_pos_x = msg->pose.position.x;
        	cur_pos_y = msg->pose.position.y;

        	double quatx = msg->pose.orientation.x;
        	double quaty = msg->pose.orientation.y;
        	double quatz = msg->pose.orientation.z;
        	double quatw = msg->pose.orientation.w;

        	tf2::Quaternion q(quatx, quaty, quatz, quatw);
        	tf2::Matrix3x3 m(q);
        	double roll, pitch, yaw;
        	m.getRPY(roll, pitch, yaw);
        	cur_yaw = yaw;

        	valid_pose = true;
        	cout << "Received pose, x= " << cur_pos_x << ", y= " << cur_pos_y << endl;
        }

        virtual BT::NodeStatus tick() override
        {

        	for (int t = 0; t < 100; t++) {
            	rclcpp::spin_some(node_);

        		if (valid_pose) {
        			break;
        		}
        		std::this_thread::sleep_for(100ms);
        		cout << "Waiting for pose..." << endl;
        	}
        	if (!valid_pose) {
        		RCLCPP_ERROR(node_->get_logger(), "Did not receive current robot pose");
        		return BT::NodeStatus::FAILURE;
        	}

        	RobotSeekGame* game = RobotSeekGame::GetRobotSeekGame();
			if (game == nullptr) {
				game = RobotSeekGame::CreateRobotSeekGame();
			}

        	std::string search_poses;
        	if (!getInput<std::string>("search_poses", search_poses)) {
        		throw BT::RuntimeError("missing search poses");
        	}

        	// Parse the ordered search pose list
        	vector<RobotSeekGame::SearchPose> poses;
            auto poses_in = BT::splitString(search_poses, ';');
            if (poses_in.size() == 0) {
                throw BT::RuntimeError("invalid input)");
            } else {
            	for (unsigned i = 0; i < poses_in.size(); i++) {
            		poses.push_back(BT::convertFromString<RobotSeekGame::SearchPose>(poses_in[i]));
            	}
            }
            if (poses.size() == 0) {
                throw BT::RuntimeError("invalid input)");
            }

            std::vector<geometry_msgs::msg::PoseStamped> path_poses;
            cout << "Parsed poses:" << endl;
            for (auto& it : poses) {
            	geometry_msgs::msg::PoseStamped msg;
            	msg.header.frame_id = "map";
            	msg.header.stamp = rclcpp::Time();
            	msg.pose.position.x = it.x;
            	msg.pose.position.y = it.y;
            	msg.pose.position.z = 0;
            	msg.pose.orientation = orientationAroundZAxis(it.yaw);
            	path_poses.push_back(msg);

                cout << "x= " << it.x << ", y= " << it.y << ", yaw= " << it.yaw << endl;
            }

            nav_msgs::msg::Path path;
            path.header.frame_id = "map";
            path.header.stamp = rclcpp::Time();
            path.poses = path_poses;
            search_path_pub_->publish(path);

        	rclcpp::spin_some(node_);

        	if (game->Init(node_, poses, cur_pos_x, cur_pos_y, cur_yaw)) {
        		// test
        		double x, y, yaw;
        		while(game->NextSearchPose(x, y, yaw));

        		return BT::NodeStatus::SUCCESS;
        	} else {
        		return BT::NodeStatus::FAILURE;
        	}
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr search_path_pub_;
        double cur_pos_x;
        double cur_pos_y;
        double cur_yaw;
        bool valid_pose;
};

