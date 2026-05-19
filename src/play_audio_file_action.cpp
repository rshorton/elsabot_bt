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

#include "play_audio_file_action.hpp"

using PlayAudioFile = elsabot_audio_output_interfaces::srv::PlayAudioFile;
using StreamType = elsabot_audio_output_interfaces::msg::StreamType;

bool PlayAudioFileAction::setRequest(Request::SharedPtr& request)
{
  std::string file_path;
  getInput<std::string>("file_path", file_path);

  std::string stream_type = "bg";
  getInput<std::string>("stream_type", stream_type);

  std::string req_id = "";
  getInput<std::string>("req_id", req_id);

  request->audio_req.file_path = file_path;

  if (stream_type == "fg") {
    request->stream_type.stream_type = StreamType::STREAM_TYPE_FG;
  } else {
    request->stream_type.stream_type = StreamType::STREAM_TYPE_BG;
  }
  request->req_id = req_id;
  return true;
}

BT::NodeStatus PlayAudioFileAction::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "PlayAudioFileAction result received, result: %s",
              response->result.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PlayAudioFileAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s; Error: %d", name().c_str(), error);  
  return BT::NodeStatus::FAILURE;
}
