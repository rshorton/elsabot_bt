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

#pragma once

#include <string>

#include <behaviortree_cpp_v3/action_node.h>

#include "robot_cat_game.hpp"

class RobotCatGameInitAction : public BT::SyncActionNode
{
    public:
    RobotCatGameInitAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout"),
                 BT::InputPort<double>("yaw_min"),
                 BT::InputPort<double>("yax_max"),
                 BT::InputPort<double>("yaw_step"),
                 BT::InputPort<double>("yaw_evade_step"),
                 BT::InputPort<double>("pitch_min"),
                 BT::InputPort<double>("pitch_max"),
                 BT::InputPort<double>("pitch_step"),
                 BT::InputPort<double>("pitch_dither_step")
        };
    }

    virtual BT::NodeStatus tick() override
    {
		double timeout = RobotCatGame::UNSET();
		getInput<double>("timeout", timeout);
       
		double yaw_min = RobotCatGame::UNSET();
		getInput<double>("yaw_min", yaw_min);

		double yax_max = RobotCatGame::UNSET();
		getInput<double>("yax_max", yax_max);

		double yaw_step = RobotCatGame::UNSET();
		getInput<double>("yaw_step", yaw_step);

		double yaw_evade_step = RobotCatGame::UNSET();
		getInput<double>("yaw_evade_step", yaw_evade_step);

		double pitch_min = RobotCatGame::UNSET();
		getInput<double>("pitch_min", pitch_min);

		double pitch_max = RobotCatGame::UNSET();
		getInput<double>("pitch_max", pitch_max);

		double pitch_step = RobotCatGame::UNSET();
		getInput<double>("pitch_step", pitch_step);

		double pitch_dither_step = RobotCatGame::UNSET();
		getInput<double>("pitch_dither_step", pitch_dither_step);

        auto game = RobotCatGame::GetRobotCatGame();
        if (game) {
            if (game->Init(timeout, yaw_min, yax_max, yaw_step, yaw_evade_step,
                           pitch_min, pitch_max, pitch_step, pitch_dither_step)) {
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
};

