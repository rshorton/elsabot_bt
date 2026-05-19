#pragma once

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

#include <stdio.h>
#include <string>

#include <behaviortree_cpp/action_node.h>

// Use this action to copy a BB variable into json
class CopyBBValueIntoJsonKeyValueAction : public BT::StatefulActionNode
{
public:
	  CopyBBValueIntoJsonKeyValueAction(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart();
    BT::NodeStatus onRunning();
    void onHalted();
};
