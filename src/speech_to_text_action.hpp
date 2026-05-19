/*
Copyright 2021 Scott Horton

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

#include "behaviortree_ros2/bt_action_node.hpp"
#include "speech_action_interfaces/action/recognize.hpp"

using namespace BT;
using Recognize = speech_action_interfaces::action::Recognize;

class SpeechToTextAction : public RosActionNode<Recognize>
{
public:
  SpeechToTextAction(const std::string& name, const NodeConfig& conf,
                     const RosNodeParams& params)
    : RosActionNode<Recognize>(name, conf, params)
  {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        { BT::InputPort<float>("start_delay"),
          BT::InputPort<unsigned int>("max_speech_duration"),
          BT::InputPort<unsigned int>("pre_speech_timeout"),
          BT::InputPort<unsigned int>("post_speech_timeout"),
          BT::OutputPort<std::string>("text") });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
