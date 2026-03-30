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

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "elsabot_audio_output_interfaces/msg/stream_type.hpp"
#include "elsabot_audio_output_interfaces/srv/play_audio_file.hpp"
#include "rclcpp/rclcpp.hpp"

class PlayAudioFileAction : public BT::SyncActionNode {
 public:
  using PlayAudioFile = elsabot_audio_output_interfaces::srv::PlayAudioFile;
  using StreamType = elsabot_audio_output_interfaces::msg::StreamType;

  PlayAudioFileAction(const std::string& name,
                      const BT::NodeConfiguration& config,
                      rclcpp::Node::SharedPtr node)
      : BT::SyncActionNode(name, config) {
    node_ = node;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("file_path"),
            BT::InputPort<std::string>("stream_type"),
            BT::InputPort<std::string>("req_id")};
  }

  virtual BT::NodeStatus tick() override {
    std::string file_path;
    getInput<std::string>("file_path", file_path);

    std::string stream_type = "bg";
    getInput<std::string>("stream_type", stream_type);

    std::string req_id = "";
    getInput<std::string>("req_id", req_id);

    auto client = node_->create_client<PlayAudioFile>("play_audio_service");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "PlayAudioFileAction, failed waiting service");
      return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<PlayAudioFile::Request>();
    request->audio_req.file_path = file_path;

    if (stream_type == "fg") {
      request->stream_type.stream_type = StreamType::STREAM_TYPE_FG;
    } else {
      request->stream_type.stream_type = StreamType::STREAM_TYPE_BG;
    }
    request->req_id = req_id;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "PlayAudioFileAction service call failed");
      client->remove_pending_request(result_future);
      return BT::NodeStatus::FAILURE;
    }

    auto result = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "PlayAudioFileAction result: %s",
                result->result.c_str());
    return BT::NodeStatus::SUCCESS;
  }

 private:
  rclcpp::Node::SharedPtr node_;
};
