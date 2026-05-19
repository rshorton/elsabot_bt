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

#include "text_to_speech_action.hpp"

bool TextToSpeechAction::setRequest(Request::SharedPtr& request)
{
  std::string text;
  getInput<std::string>("text", text);

  RCLCPP_INFO(logger(), "TextToSpeechAction, saying: [%s]", text.c_str());

  if (text.empty()) {
    return false;
  }

  std::string req_id = "";
  getInput<std::string>("req_id", req_id);

  request->tts_req.text = text;
  request->req_id = req_id;

  return true;
}

BT::NodeStatus TextToSpeechAction::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "TextToSpeechAction result received, result: %s",
              response->result.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TextToSpeechAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s; Error: %d", name().c_str(), error);
  return BT::NodeStatus::FAILURE;
}
