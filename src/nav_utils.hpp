/*
Copyright 2023 Scott Horton

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

#ifndef _NAV_UTILS_HPP_
#define _NAV_UTILS_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/geometry_utils.hpp"

bool calculate_pose_orientations(std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>);

#endif //_NAV_UTILS_HPP_