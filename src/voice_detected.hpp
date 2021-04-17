#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <behaviortree_cpp_v3/action_node.h>

class VoiceDetected : public BT::SyncActionNode
{
    public:
		VoiceDetected(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config),
	        vad(false)
        {
            node_ = rclcpp::Node::make_shared("voice_activity_det_node");

            vad_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
              "speech_detect/vad",
              rclcpp::SystemDefaultsQoS(),
              std::bind(&VoiceDetected::vadCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts()
        {
        	return{};
        }

        virtual BT::NodeStatus tick() override
        {
        	rclcpp::spin_some(node_);
            if (vad) {
            	return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        void vadCallback(std_msgs::msg::Bool::SharedPtr msg)
        {
        	vad = msg->data;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vad_sub_;
        bool vad;
};
