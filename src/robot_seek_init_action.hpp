#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <behaviortree_cpp_v3/action_node.h>

#include "robot_seek_game.hpp"

using namespace std;

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
        	rclcpp::spin_some(node_);

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

        	if (game->Init(node_, search_poses, cur_pos_x, cur_pos_y, cur_yaw)) {
        		return BT::NodeStatus::SUCCESS;
        	} else {
        		return BT::NodeStatus::FAILURE;
        	}
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

        double cur_pos_x;
        double cur_pos_y;
        double cur_yaw;
        bool valid_pose;
};

