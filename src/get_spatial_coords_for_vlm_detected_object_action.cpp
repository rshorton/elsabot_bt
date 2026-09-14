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

#include "transform_helper.hpp"

#include "get_spatial_coords_for_vlm_detected_object_action.hpp"

bool GetSpatialCoordsForVLMDetectedObjectAction::setRequest(Request::SharedPtr& request)
{
  uint32_t roi_x1;
  uint32_t roi_y1;
  uint32_t roi_x2;
  uint32_t roi_y2;
  uint32_t roi_image_w;
  uint32_t roi_image_h;

  getInput<uint32_t>("roi_x1", roi_x1);
  getInput<uint32_t>("roi_y1", roi_y1);
  getInput<uint32_t>("roi_x2", roi_x2);
  getInput<uint32_t>("roi_y2", roi_y2);
  getInput<uint32_t>("roi_image_w", roi_image_w);
  getInput<uint32_t>("roi_image_h", roi_image_h);

  std::string image_time_str;
  getInput<std::string>("image_time", image_time_str);

  RCLCPP_INFO(logger(), "%s, image time: %s", name().c_str(), image_time_str.c_str());

  auto time_stamp_msg = BT::convertFromString<builtin_interfaces::msg::Time>(image_time_str);
  image_time_ = rclcpp::Time(time_stamp_msg);
  RCLCPP_INFO(logger(), "%s, time: %ld", name().c_str(), image_time_.nanoseconds());

  request->time_stamp = time_stamp_msg;
  if (roi_x1 > roi_x2) {
    auto t = roi_x1;
    roi_x1 = roi_x2;
    roi_x2 = t;
  }
  if (roi_y1 > roi_y2) {
    auto t = roi_y1;
    roi_y1 = roi_y2;
    roi_y2 = t;
  }
  request->roi_x = (float)roi_x1/float(roi_image_w);
  request->roi_y = (float)roi_y1/float(roi_image_h);
  request->roi_w = (float)(roi_x2 - roi_x1)/float(roi_image_w);
  request->roi_h = (float)(roi_y2 - roi_y1)/float(roi_image_h);
   
  return true;
}

BT::NodeStatus GetSpatialCoordsForVLMDetectedObjectAction::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "%s, valid: %d, x,y,z: %f, %f, %f, frame_id: %s",
              name().c_str(), response->valid, response->x, response->y, response->z, response->frame_id.c_str());

  if (!response->valid) {
    return BT::NodeStatus::FAILURE; 
  }

  std::string output_frame_id;
  getInput<std::string>("output_frame_id", output_frame_id);
  if (output_frame_id.empty()) {
    RCLCPP_ERROR(logger(), "%s, no output frame specified", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto helper = TransformHelper::GetInstance();
  if (!helper) {
    return BT::NodeStatus::FAILURE;
  }

  double x = response->x;
  double y = response->y;
  double z = response->z;

  if (!helper->Transform(response->frame_id, output_frame_id, x, y, z, image_time_)) {
    RCLCPP_ERROR(logger(), "%s, failed to transform coords", name().c_str());
    return BT::NodeStatus::FAILURE;
  }				
 
  RCLCPP_INFO(logger(), "%s, x,y,z: %f, %f, %f", name().c_str(), x, y, z);

  setOutput("x", x);
  setOutput("y", y);
  setOutput("z", z);

  //Pose3D pose(x, y, z);
  //setOutput("roi_coords", pose);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetSpatialCoordsForVLMDetectedObjectAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s, Error: %d", name().c_str(), error);  
  return BT::NodeStatus::FAILURE;
}
