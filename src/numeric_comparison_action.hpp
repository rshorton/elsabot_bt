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

using namespace std;

class NumericComparisonAction : public BT::SyncActionNode
{
    public:
		NumericComparisonAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<std::string>("comparison"),
                    BT::InputPort<double>("a"),
                    BT::InputPort<double>("b")
                   };
        }

        virtual BT::NodeStatus tick() override
        {
    		string comparison;
            double a, b;

			if (!getInput<std::string>("comparison", comparison)) {
				throw BT::RuntimeError("missing type option");
			}

			if (!getInput<double>("a", a)) {
				throw BT::RuntimeError("missing a");
			}

			if (!getInput<double>("b", b)) {
				throw BT::RuntimeError("missing b");
			}

            if (comparison == "gt") {
                if (a > b) {
                    return BT::NodeStatus::SUCCESS;
                }
            } else if (comparison == "eq") {
                if (a == b) {
                    return BT::NodeStatus::SUCCESS;
                }
            } else if (comparison == "lt") {
                if (a < b) {
                    return BT::NodeStatus::SUCCESS;
                }
            }
            return BT::NodeStatus::FAILURE;
        }

    private:
};
