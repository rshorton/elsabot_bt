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

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_head_interfaces/msg/track_cmd.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Singleton for publishing the Track control state - shared by all TrackActionNode instances
class TrackActionROSNodeIf
{
public:
	TrackActionROSNodeIf(TrackActionROSNodeIf const&) = delete;
	TrackActionROSNodeIf& operator=(TrackActionROSNodeIf const&) = delete;

    static std::shared_ptr<TrackActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
        static std::shared_ptr<TrackActionROSNodeIf> s{new TrackActionROSNodeIf(node)};
        return s;
    }

    void setTrackSettings(const std::string &mode, const std::string &rate, const std::string sound_track_mode,
						  bool turn_base, const std::string &object_type, double min_confidence)
    {
    	RCLCPP_INFO(node_->get_logger(), "Set track: mode= %s, rate= %s, sound_track_mode= %s, \
					turn_base= %d, object_type= %s, min_confidence= %f",
    				mode.c_str(), rate.c_str(), sound_track_mode.c_str(), turn_base, object_type.c_str(),
					min_confidence);
    	auto msg = robot_head_interfaces::msg::TrackCmd();
    	msg.mode = mode;
// Fix rename rate to scan_step		
        msg.scan_step = rate;
        msg.sound_mode = sound_track_mode;
        msg.turn_base = turn_base;
		msg.object_type = object_type;
		msg.min_confidence = min_confidence;
        track_publisher_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }

private:
    TrackActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
    	track_publisher_ = node_->create_publisher<robot_head_interfaces::msg::TrackCmd>("/head/track_cmd", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::TrackCmd>::SharedPtr track_publisher_;
};

class TrackAction : public BT::SyncActionNode
{
    public:
		TrackAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = TrackActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("mode"),				// Off, Scan, TrackScan, Manual, LookDown
        			 BT::InputPort<std::string>("rate"),				// Scan rate
					 BT::InputPort<std::string>("sound_track_mode"),	// 'any', 'wakeword', 'none'
        			 BT::InputPort<bool>("turn_base"),					// Turn base to face tracked object
					 BT::InputPort<std::string>("object_type"),			// Object type to track
	       			 BT::InputPort<double>("min_confidence")};			// Min detection confidence for tracking

        }

        virtual BT::NodeStatus tick() override
        {
        	std::string mode;
        	std::string rate;
			std::string object_type = "person";
        	std::string sound_track_mode;
        	bool turn_base = false;
			double min_confidence = 0.7;

        	if (!getInput<std::string>("mode", mode)) {
    			throw BT::RuntimeError("missing mode");
    		}
        	if (!getInput<std::string>("rate", rate)) {
    			throw BT::RuntimeError("missing rate");
    		}
        	if (!getInput<std::string>("sound_track_mode", sound_track_mode)) {
    			throw BT::RuntimeError("missing sound_track_mode");
    		}
        	if (!getInput<bool>("turn_base", turn_base)) {
    			throw BT::RuntimeError("missing turn_base");
    		}
        	getInput<std::string>("object_type", object_type);
        	getInput<double>("min_confidence", min_confidence);

        	node_if_->setTrackSettings(mode, rate, sound_track_mode, turn_base, object_type, min_confidence);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<TrackActionROSNodeIf> node_if_;
};
