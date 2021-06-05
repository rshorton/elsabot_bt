#pragma once

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/track.hpp"
#include <behaviortree_cpp_v3/action_node.h>

class TrackAction : public BT::SyncActionNode
{
    public:
		TrackAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("bt_track_action_node");

            track_publisher_ = node_->create_publisher<face_control_interfaces::msg::Track>("/head/track", 2);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("mode"),
        			 BT::InputPort<std::string>("rate"),
					 BT::InputPort<bool>("detect_voice")};
        }

        virtual BT::NodeStatus tick() override
        {
        	std::string mode;
        	std::string rate;
        	bool detect_voice = false;
        	if (!getInput<std::string>("mode", mode)) {
    			throw BT::RuntimeError("missing mode");
    		}
        	if (!getInput<std::string>("rate", rate)) {
    			throw BT::RuntimeError("missing rate");
    		}
        	if (!getInput<bool>("detect_voice", detect_voice)) {
    			throw BT::RuntimeError("missing detect_voice");
    		}

        	RCLCPP_INFO(node_->get_logger(), "Set track: mode= %s, rate= %s", mode.c_str(), rate.c_str());
        	auto message = face_control_interfaces::msg::Track();
            message.mode = mode;
            message.rate = rate;
            message.voice_detect = detect_voice;
            track_publisher_->publish(message);

            std::this_thread::sleep_for(100ms);

            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<face_control_interfaces::msg::Track>::SharedPtr track_publisher_;
};
