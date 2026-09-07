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

#include <behaviortree_ros2/bt_service_node.hpp>

#include "bt_custom_type_helpers.hpp"
#include "dart_gun_interfaces/srv/fire_at_specified_target.hpp"

using FireAtSpecifiedTarget = dart_gun_interfaces::srv::FireAtSpecifiedTarget;

class DartGunFireAtSpecifiedTargetAction : public BT::RosServiceNode<FireAtSpecifiedTarget>
{
public:
  explicit DartGunFireAtSpecifiedTargetAction(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
    : RosServiceNode<FireAtSpecifiedTarget>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<int>("count"),
          BT::InputPort<std::string>("frame_id"),
          BT::InputPort<Pose3D>("target") });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

