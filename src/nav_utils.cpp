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

#include "nav_utils.hpp"

bool calculate_pose_orientations(std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>> poses)
{
    auto num_poses = poses->size();
    if (num_poses == 1) {
        return true;
    }

    tf2::Quaternion q;

    // Set the orientation of each post such that it points to the next pose
    for (size_t i = 0; i + 1 < num_poses; i++) {
        double dx = (*poses)[i + 1].pose.position.x - (*poses)[i].pose.position.x;
        double dy = (*poses)[i + 1].pose.position.y - (*poses)[i].pose.position.y;

    	double yaw = std::atan2(dy, dx);
        q.setRPY(0.0, 0.0, yaw);
        (*poses)[i].pose.orientation = tf2::toMsg(q);

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "dx: %f, dy: %f, yaw: %f", dx, dy, yaw);

    }

    // Last point uses same orientation as last.
    // FIX - allow this to be specified.
    (*poses)[num_poses - 1].pose.orientation = tf2::toMsg(q);

    return true;
}
