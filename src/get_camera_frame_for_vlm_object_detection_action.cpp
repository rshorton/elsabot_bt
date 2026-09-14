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

#include <sstream>
#include "base64.hpp"
#include "bt_custom_type_helpers.hpp"

#include "get_camera_frame_for_vlm_object_detection_action.hpp"

bool GetCameraFrameForVLMObjectDetectionAction::setRequest(Request::SharedPtr& request)
{
  return true;
}

BT::NodeStatus GetCameraFrameForVLMObjectDetectionAction::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "%s, valid: %d, image Format: %s, Size: %zu bytes",
              name().c_str(), response->valid, response->image.format.c_str(), response->image.data.size());
  if (!response->valid) {
    return BT::NodeStatus::FAILURE; 
  }

  std::string base64_image = base64_encode(response->image.data);
  RCLCPP_INFO(logger(), "%s, Base64 string size: %zu", name().c_str(), base64_image.size());

  auto image = "data:image/jpeg;base64," + base64_image;
  setOutput("base64_image", image);

  auto image_time = convertToString(response->image.header.stamp);
  RCLCPP_INFO(logger(), "%s, image_time: %s", name().c_str(), image_time.c_str());

  setOutput("image_time", image_time);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetCameraFrameForVLMObjectDetectionAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s, Error: %d", name().c_str(), error);  
  return BT::NodeStatus::FAILURE;
}
