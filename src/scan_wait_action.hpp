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

#include "ros_common.hpp"
#include "robot_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_head_interfaces/msg/scan_status.hpp"

#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

class ScanWaitAction: public CoroActionNode
{
public:
  	ScanWaitAction(const std::string& name):
        CoroActionNode(name, {})
    {
    }

private:
    NodeStatus tick() override
    {
        rclcpp::Node::SharedPtr node = ROSCommon::GetInstance()->GetNode();

        int initialized = false;
        robot_head_interfaces::msg::TrackStatus cur_status;
    	robot_head_interfaces::msg::TrackStatus start_status;

        while (true) {

            if (!RobotStatus::GetInstance()->GetTrackStatus(cur_status)) {
                RCLCPP_WARN(node->get_logger(), "ScanWaitAction, track status not available");
                setStatusRunningAndYield();
                continue;
            }

            if (!initialized) {
                start_status = cur_status;
                RCLCPP_INFO(node->get_logger(), "lcnt= %d, rcnt= %d",
                    start_status.scan_status.at_left_count, start_status.scan_status.at_right_count);
                initialized = true;

            } else {
                if (start_status.scan_status.at_left_count != cur_status.scan_status.at_left_count &&
                    start_status.scan_status.at_right_count != cur_status.scan_status.at_right_count) {
                    return NodeStatus::SUCCESS;
                }
            }

            // set status to RUNNING and "pause/sleep"
            // If halt() is called, we will not resume execution (stack destroyed)
            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << name() <<": Halted." << std::endl;
        // Do not forget to call this at the end.
        CoroActionNode::halt();
    }

};
