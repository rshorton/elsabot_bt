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

#include "dart_gun_disable_tracking_action.hpp"

bool DartGunDisableTrackingAction::setRequest(Request::SharedPtr& request)
{
  return true;
}

BT::NodeStatus DartGunDisableTrackingAction::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(logger(), "DartGunDisableTrackingAction result received, result: %d",
              response->result);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DartGunDisableTrackingAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s; Error: %d", name().c_str(), error);  
  return BT::NodeStatus::FAILURE;
}
