#pragma once

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/smile.hpp"
#include <behaviortree_cpp_v3/action_node.h>

class SmileAction : public BT::SyncActionNode
{
    public:
		SmileAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("bt_smile_action_node");

            smile_publisher_ = node_->create_publisher<face_control_interfaces::msg::Smile>("/head/smile", 2);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("level"), BT::InputPort<std::string>("duration_ms") };
        }

        virtual BT::NodeStatus tick() override
        {
        	int32_t level = 2;
        	int32_t duration = 2000;
        	if (!getInput<std::int32_t>("level", level)) {
    			throw BT::RuntimeError("missing level");
    		}
        	if (!getInput<std::int32_t>("duration_ms", duration)) {
    			throw BT::RuntimeError("missing duration");
    		}

        	RCLCPP_INFO(node_->get_logger(), "Set smile: level= %d, duration= %d", level, duration);
        	auto message = face_control_interfaces::msg::Smile();
            message.mode = "smile";
            message.level = level;
            message.duration_ms = duration;
            message.use_as_default = false;
            smile_publisher_->publish(message);

            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<face_control_interfaces::msg::Smile>::SharedPtr smile_publisher_;
};
