#pragma once

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/antenna.hpp"
#include <behaviortree_cpp_v3/action_node.h>

class AntennaAction : public BT::SyncActionNode
{
    public:
	AntennaAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("bt_antenna_action_node");

            antenna_publisher_ = node_->create_publisher<face_control_interfaces::msg::Antenna>("/head/antenna", 2);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("left_blink_pattern"), BT::InputPort<std::string>("right_blink_pattern"),
        		     BT::InputPort<uint32_t>("rate"), BT::InputPort<uint32_t>("intensity")};
        }

        virtual BT::NodeStatus tick() override
        {
        	uint32_t rate;
        	uint32_t intensity;
        	std::string left_blink_pattern;
        	std::string right_blink_pattern;

        	if (!getInput<uint32_t>("rate", rate)) {
    			throw BT::RuntimeError("missing rate");
    		}
        	if (!getInput<uint32_t>("intensity", intensity)) {
    			throw BT::RuntimeError("missing intensity");
    		}
        	if (!getInput<std::string>("left_blink_pattern", left_blink_pattern)) {
    			throw BT::RuntimeError("missing left_blink_pattern");
    		}
        	if (!getInput<std::string>("right_blink_pattern", right_blink_pattern)) {
    			throw BT::RuntimeError("missing right_blink_pattern");
    		}

        	RCLCPP_INFO(node_->get_logger(), "Set Antenna: rate= %d, intensity= %d, left pattern= %s, right pattern= %s",
        				rate, intensity, left_blink_pattern.c_str(), right_blink_pattern.c_str());

        	auto message = face_control_interfaces::msg::Antenna();
            message.rate = rate;
            message.intensity = intensity;
            message.left_blink_pattern = left_blink_pattern;
            message.right_blink_pattern = right_blink_pattern;
            antenna_publisher_->publish(message);

            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<face_control_interfaces::msg::Antenna>::SharedPtr antenna_publisher_;
};
