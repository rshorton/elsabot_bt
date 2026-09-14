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

#include "bt_custom_type_helpers.hpp"
#include "robot_head_interfaces/srv/get_spatial_coords.hpp"

using GetSpatialCoords = robot_head_interfaces::srv::GetSpatialCoords;

class GetSpatialCoordsForVLMDetectedObjectAction : public BT::RosServiceNode<GetSpatialCoords>
{
public:
  explicit GetSpatialCoordsForVLMDetectedObjectAction(const std::string& name, const BT::NodeConfig& conf,
                                             const BT::RosNodeParams& params)
    : RosServiceNode<GetSpatialCoords>(name, conf, params)
  {}

  static BT::PortsList providedPorts() {
        return {BT::InputPort<uint32_t>("roi_x1"),
                BT::InputPort<uint32_t>("roi_y1"),
                BT::InputPort<uint32_t>("roi_x2"),
                BT::InputPort<uint32_t>("roi_y2"),
                BT::InputPort<uint32_t>("roi_image_w"),
                BT::InputPort<uint32_t>("roi_image_h"),
                BT::InputPort<std::string>("image_time"),
                BT::InputPort<std::string>("output_frame_id"),                
                BT::OutputPort<double>("x"),
                BT::OutputPort<double>("y"),
                BT::OutputPort<double>("z")};
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  rclcpp::Time image_time_;
};

