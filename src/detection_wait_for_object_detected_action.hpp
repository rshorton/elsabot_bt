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

/*
DetectionWaitForObjectDetectedAction

Waits for the specified object to become detected

Inputs:
det - object detector instance to use
obj_class - class of object to detect
timeout_ms - timeout in milliseconds

Outputs:
SUCCESS - found
FAILURE - not foud
*/

#pragma once

#include <string>
#include <chrono>

#include <behaviortree_cpp/action_node.h>


class DetectionWaitForObjectDetectedAction : public BT::StatefulActionNode
{
public:
	DetectionWaitForObjectDetectedAction(const std::string& name, const BT::NodeConfig& config) :
		BT::StatefulActionNode(name, config)
    {}

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("det"),
			BT::InputPort<std::string>("obj_class"),
			BT::InputPort<double>("timeout_sec")
		};
	}

	BT::NodeStatus onStart();
    BT::NodeStatus onRunning();
    void onHalted();

private:
	std::string detector_;
	std::string obj_class_;
	double timeout_sec_{0.0};

  	std::chrono::time_point<std::chrono::steady_clock> start_time_;

};
