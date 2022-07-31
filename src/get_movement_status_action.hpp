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

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include "imu_topic.hpp"

using namespace std;

class GetMovementStatusAction : public BT::SyncActionNode
{
    public:
		GetMovementStatusAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<std::string>("type"),
                    BT::OutputPort<double>("x"),
                    BT::OutputPort<double>("y"),
                    BT::OutputPort<double>("z")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
    		string type;

			if (!getInput<std::string>("type", type)) {
				throw BT::RuntimeError("missing type option");
			}

            IMUTopic* imu_topic =  IMUTopic::GetInstance();
            if (imu_topic) {
                if (type == "angular_velocity") {
                    double x, y, z;
                    if (imu_topic->GetAngularVelocity(x, y, z)) {
                        setOutput("x", x);
                        setOutput("y", y);
                        setOutput("z", z);
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetMovementAction: angular, x,y,z: %f, %f, %f", x, y, z);
                        return BT::NodeStatus::SUCCESS;
                    }
                }
            }                
            return BT::NodeStatus::FAILURE;
        }

    private:
};
