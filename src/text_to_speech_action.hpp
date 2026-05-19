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
#include "elsabot_audio_output_interfaces/srv/play_tts.hpp"

using PlayTTS = elsabot_audio_output_interfaces::srv::PlayTTS;

class TextToSpeechAction : public BT::RosServiceNode<PlayTTS>
{
public:
  explicit TextToSpeechAction(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
    : RosServiceNode<PlayTTS>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>("text"),
          BT::InputPort<std::string>("req_id") });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
