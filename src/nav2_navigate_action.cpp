#include <cmath>

#include "nav2_navigate_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "bt_custom_type_helpers.hpp"
#include "robot_status.hpp"

using nav2_util::geometry_utils::orientationAroundZAxis;

bool Nav2NavigateAction::setGoal(RosActionNode::Goal& goal_msg)
{
  // optional
  std::string behavior_tree;
  getInput<std::string>("behavior_tree", behavior_tree);

  // Support either an absolute goal or a relative goal
  Pose2D goal;
  Pose2DRelative goal_relative;
  if (!getInput<Pose2D>("goal", goal)) {
     if (!getInput<Pose2DRelative>("goal_relative", goal_relative)) {
        throw BT::RuntimeError("missing required input [goal or goal_relative]");
     }     
     if (!calc_position_and_yaw_from_relative(goal_relative, goal)) {
        RCLCPP_ERROR(logger(), "Failed calculate new position from relative goal.");
        return false;
     }
  }

  RCLCPP_INFO(logger(), "Position goal %f %f %f", goal.x, goal.y, goal.yaw);

  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose.position.x = goal.x;
  goal_msg.pose.pose.position.y = goal.y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation = orientationAroundZAxis(goal.yaw/180.0*M_PI);
  goal_msg.behavior_tree = behavior_tree;

  RCLCPP_INFO(logger(), "Orientation goal %f %f %f %f", goal_msg.pose.pose.orientation.x,
              goal_msg.pose.pose.orientation.y, goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w);
  return true;
}

NodeStatus Nav2NavigateAction::onResultReceived(const RosActionNode::WrappedResult&)
{
  RCLCPP_INFO(logger(), "%s: Nav2NavigateAction finished", name().c_str());
  return NodeStatus::SUCCESS;
}

NodeStatus Nav2NavigateAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void Nav2NavigateAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

bool Nav2NavigateAction::calc_position_and_yaw_from_relative(const Pose2DRelative pose_relative, Pose2D &pose)
{
    RobotStatus* robot_status = RobotStatus::GetInstance();
    if (!robot_status) {
        RCLCPP_ERROR(logger(), "Failed to get robot status for getting current pose");
        return false;
    }

    Pose2D cur_pose;
    if (!robot_status->GetPose2D(cur_pose)) {
        RCLCPP_ERROR(logger(), "Failed to get current pose");
        return false;
    }

    auto angle = pose_relative.heading + cur_pose.yaw;
    angle = std::remainder(angle, 360.0);
    if (angle > 180.0) {
        angle = angle - 360.0;
    } else if (angle < -180.0) {
        angle = angle + 360.0;
    }

    angle = angle*M_PI/180.0;
    auto delta_x = cos(angle)*pose_relative.distance;
    auto delta_y = sin(angle)*pose_relative.distance;

    RCLCPP_INFO(logger(), "Goal deltas, x: %f, y: %f, heading: %f", delta_x, delta_y, angle/M_PI*180.0);

    pose.x = cur_pose.x + delta_x;
    pose.y = cur_pose.y + delta_y;
    pose.yaw = angle*180.0/M_PI;
    return true;        
}

// Plugin registration.
// The class Nav2NavigateAction will self register with name  "Nav2NavigateAction".
//CreateRosNodePlugin(Nav2NavigateAction, "Nav2NavigateAction");



