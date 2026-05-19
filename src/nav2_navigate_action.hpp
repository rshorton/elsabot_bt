#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "bt_custom_type_helpers.hpp"

using namespace BT;

class Nav2NavigateAction : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  Nav2NavigateAction(const std::string& name, const NodeConfig& conf,
                     const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        { InputPort<Pose2D>("goal"),
          InputPort<Pose2DRelative>("goal_relative"),
          InputPort<std::string>("behavior_tree")});
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;

private:  
  bool calc_position_and_yaw_from_relative(const Pose2DRelative pose_relative, Pose2D &pose);
};
