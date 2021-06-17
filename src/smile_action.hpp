#pragma once

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/smile.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Single for publishing the smile control state - shared by all SmileActionNode instances
class SmileActionROSNodeIf
{
public:
	SmileActionROSNodeIf(SmileActionROSNodeIf const&) = delete;
	SmileActionROSNodeIf& operator=(SmileActionROSNodeIf const&) = delete;

    static std::shared_ptr<SmileActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<SmileActionROSNodeIf> s{new SmileActionROSNodeIf(node)};
        return s;
    }

    void setSmileState(int32_t level, int32_t duration)
    {

    	RCLCPP_INFO(node_->get_logger(), "Set smile: level= %d, duration= %d", level, duration);
    	auto message = face_control_interfaces::msg::Smile();
        message.mode = "smile";
        message.level = level;
        message.duration_ms = duration;
        message.use_as_default = false;
        smile_publisher_->publish(message);
    }

private:
    SmileActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        smile_publisher_ = node_->create_publisher<face_control_interfaces::msg::Smile>("/head/smile", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<face_control_interfaces::msg::Smile>::SharedPtr smile_publisher_;
};

class SmileAction : public BT::SyncActionNode
{
    public:
		SmileAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
            node_if_ = SmileActionROSNodeIf::instance(node);
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

        	node_if_->setSmileState(level, duration);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<SmileActionROSNodeIf> node_if_;
};
