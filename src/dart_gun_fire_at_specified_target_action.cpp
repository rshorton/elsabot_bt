/*
Copyright 2026 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

                http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "dart_gun_fire_at_specified_target_action.hpp"

bool DartGunFireAtSpecifiedTargetAction::setRequest(Request::SharedPtr& request)
{
  int count;
  getInput<int>("count", count);
  if (count <= 0) {
    return false;
  }

  std::string frame_id;
  getInput<std::string>("frame_id", frame_id);
  if (frame_id.empty()) {
    return false;
  }

  Pose3D target;
  getInput<Pose3D>("target", target);

  request->count = count;

  request->target.header.frame_id = frame_id;
  
  auto node = node_.lock();
  if (node) {
    request->target.header.stamp = node->get_clock()->now();
  }    
  request->target.pose.position.x = target.x;
  request->target.pose.position.y = target.y;
  request->target.pose.position.z = target.z;
  request->target.pose.orientation.x = 0.0;
  request->target.pose.orientation.y = 0.0;
  request->target.pose.orientation.z = 0.0;
  request->target.pose.orientation.w = 1.0;

  return true;
}

BT::NodeStatus DartGunFireAtSpecifiedTargetAction::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "DartGunFireAtSpecifiedTargetAction result received, result: %d",
              response->cmd_result);
  if (response->cmd_result == FireAtSpecifiedTarget::Response::CMD_SUCCESS) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }    
}

BT::NodeStatus DartGunFireAtSpecifiedTargetAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s; Error: %d", name().c_str(), error);  
  return BT::NodeStatus::FAILURE;
}
