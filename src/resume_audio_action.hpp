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
#include <iostream>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "elsabot_audio_output_interfaces/msg/stream_type.hpp"
#include "elsabot_audio_output_interfaces/srv/resume_audio.hpp"
#include "rclcpp/rclcpp.hpp"

class ResumeAudioAction : public BT::SyncActionNode {
 public:
  using ResumeAudio = elsabot_audio_output_interfaces::srv::ResumeAudio;
  using StreamType = elsabot_audio_output_interfaces::msg::StreamType;

  ResumeAudioAction(const std::string& name,
                    const BT::NodeConfiguration& config,
                    rclcpp::Node::SharedPtr node)
      : BT::SyncActionNode(name, config) {
    node_ = node;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("stream_type")};
  }

  virtual BT::NodeStatus tick() override {
    std::string stream_type = "bg";
    getInput<std::string>("stream_type", stream_type);

    auto client = node_->create_client<ResumeAudio>("resume_audio");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "ResumeAudioAction, failed waiting for service");
      return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<ResumeAudio::Request>();
    if (stream_type == "fg") {
      request->stream_type.stream_type = StreamType::STREAM_TYPE_FG;
    } else {
      request->stream_type.stream_type = StreamType::STREAM_TYPE_BG;
    }

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "ResumeAudioAction service call failed");
      client->remove_pending_request(result_future);
      return BT::NodeStatus::FAILURE;
    }

    auto result = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "ResumeAudioAction result: %s",
                result->result.c_str());
    return BT::NodeStatus::SUCCESS;
  }

 private:
  rclcpp::Node::SharedPtr node_;
};
