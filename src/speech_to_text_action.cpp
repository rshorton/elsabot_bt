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

#include "speech_to_text_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SpeechToTextAction::setGoal(RosActionNode::Goal& goal)
{
  unsigned int max_speech_duration = 10;
  getInput<unsigned int>("max_speech_duration", max_speech_duration);

  unsigned int pre_speech_timeout = 2;
  getInput<unsigned int>("pre_speech_timeout", pre_speech_timeout);

  unsigned int post_speech_timeout = 2;
  getInput<unsigned int>("post_speech_timeout", post_speech_timeout);

  float start_delay = 0.0f;
  getInput<float>("start_delay", start_delay);

  RCLCPP_INFO(logger(), "Sending speech-to-text goal, start_delay: %f", start_delay);

  goal.max_speech_duration = max_speech_duration;
  goal.pre_speech_timeout = pre_speech_timeout;
  goal.post_speech_timeout = post_speech_timeout;
  goal.start_delay = start_delay;

  return true;
}

NodeStatus SpeechToTextAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::string text = wr.result->text;
  RCLCPP_INFO(logger(), "%s: Recognized [%s]", name().c_str(), text.c_str());
  if (!text.compare("ERROR")) {
     text = "";
  }
  setOutput("text", text);
  return NodeStatus::SUCCESS;
}

NodeStatus SpeechToTextAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void SpeechToTextAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}
