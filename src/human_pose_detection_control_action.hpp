#pragma once

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "human_pose_interfaces/msg/enable_pose_detection.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Single for publishing the pose detection control state - shared by all PoseDetectionControlAction instances
class PoseDetectionControlActionROSNodeIf
{
public:
	PoseDetectionControlActionROSNodeIf(PoseDetectionControlActionROSNodeIf const&) = delete;
	PoseDetectionControlActionROSNodeIf& operator=(PoseDetectionControlActionROSNodeIf const&) = delete;

    static std::shared_ptr<PoseDetectionControlActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<PoseDetectionControlActionROSNodeIf> s{new PoseDetectionControlActionROSNodeIf(node)};
        return s;
    }

    void setPoseDetectionControlState(bool enable)
    {
        std::this_thread::sleep_for(1000ms);
    	RCLCPP_INFO(node_->get_logger(), "Set pose detection control: enable= %d", enable);
    	auto message = human_pose_interfaces::msg::EnablePoseDetection();
        message.enable = enable;
        pub_->publish(message);
        std::this_thread::sleep_for(1000ms);
    }

private:
    PoseDetectionControlActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        pub_ = node_->create_publisher<human_pose_interfaces::msg::EnablePoseDetection>("/head/enable_pose_detect", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<human_pose_interfaces::msg::EnablePoseDetection>::SharedPtr pub_;
};

class PoseDetectionControlAction : public BT::SyncActionNode
{
    public:
		PoseDetectionControlAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = PoseDetectionControlActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<bool>("enable") };
        }

        virtual BT::NodeStatus tick() override
        {
        	bool enable;
        	if (!getInput<bool>("enable", enable)) {
    			throw BT::RuntimeError("missing enable");
    		}

        	node_if_->setPoseDetectionControlState(enable);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<PoseDetectionControlActionROSNodeIf> node_if_;
};
