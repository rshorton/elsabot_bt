#pragma once

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/head_tilt.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Single for publishing the HeadTilt control state - shared by all HeadTiltActionNode instances
class HeadTiltActionROSNodeIf
{
public:
	HeadTiltActionROSNodeIf(HeadTiltActionROSNodeIf const&) = delete;
	HeadTiltActionROSNodeIf& operator=(HeadTiltActionROSNodeIf const&) = delete;

    static std::shared_ptr<HeadTiltActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
        static std::shared_ptr<HeadTiltActionROSNodeIf> s{new HeadTiltActionROSNodeIf(node)};
        return s;
    }

    void setHeadTiltState(int32_t angle, int32_t dwell_ms)
    {
    	RCLCPP_INFO(node_->get_logger(), "Set head tilt: angle= %d, dwell_ms= %d", angle, dwell_ms);
    	auto message = face_control_interfaces::msg::HeadTilt();
        message.angle = angle;
        message.dwell_duration = dwell_ms;
        message.transition_duration = 0;
        head_tilt_publisher_->publish(message);
    }

private:
    HeadTiltActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        head_tilt_publisher_ = node_->create_publisher<face_control_interfaces::msg::HeadTilt>("/head/tilt", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<face_control_interfaces::msg::HeadTilt>::SharedPtr head_tilt_publisher_;
};

class HeadTiltAction : public BT::SyncActionNode
{
    public:
		HeadTiltAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = HeadTiltActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("angle"), BT::InputPort<std::string>("dwell_ms") };
        }

        virtual BT::NodeStatus tick() override
        {
        	int32_t angle = 2;
        	int32_t dwell_ms = 2000;
        	if (!getInput<std::int32_t>("angle", angle)) {
    			throw BT::RuntimeError("missing angle");
    		}
        	if (!getInput<std::int32_t>("dwell_ms", dwell_ms)) {
    			throw BT::RuntimeError("missing dwell_ms");
    		}

        	node_if_->setHeadTiltState(angle, dwell_ms);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<HeadTiltActionROSNodeIf> node_if_;
};
