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

#pragma once

#include <string>

#include "builtin_interfaces/msg/time.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>
#include "robot_head_interfaces/srv/rgbd_snapshot.hpp"

// This action is used to get a color image snapshot from the head
// camera.  When the snapshot is taken, a synchronized depth frame
// is buffered on the robot_head node for a period of time before
// being purged.  The GetSpatialCoordsForCameraFrameROI action
// can then be used to calculate the spatial coordinates corresponding
// to a region of interest.  The use case is for determining the spatial
// coords of objects identified by a VLM model.

using RGBDSnapShot = robot_head_interfaces::srv::RGBDSnapshot;

class GetCameraFrameForVLMObjectDetectionAction : public BT::RosServiceNode<RGBDSnapShot>
{
public:
  explicit GetCameraFrameForVLMObjectDetectionAction(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
    : RosServiceNode<RGBDSnapShot>(name, conf, params)
  {}

  static BT::PortsList providedPorts() {
        return {BT::OutputPort<std::string>("base64_image"),
                BT::OutputPort<std::string>("image_time")};
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

