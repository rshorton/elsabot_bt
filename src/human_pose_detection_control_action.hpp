#pragma once

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "human_pose_interfaces/msg/enable_pose_detection.hpp"
#include <behaviortree_cpp_v3/action_node.h>

class PoseDetectionControlAction : public BT::SyncActionNode
{
    public:
		PoseDetectionControlAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("bt_pose_detection_control_action_node");

            pub_ = node_->create_publisher<human_pose_interfaces::msg::EnablePoseDetection>("/head/enable_pose_detect", 2);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("enable") };
        }

        virtual BT::NodeStatus tick() override
        {
        	bool enable;
        	if (!getInput<bool>("enable", enable)) {
    			throw BT::RuntimeError("missing enable");
    		}

            std::this_thread::sleep_for(1000ms);

        	RCLCPP_INFO(node_->get_logger(), "Set pose detection: enable= %d", enable);
        	auto message = human_pose_interfaces::msg::EnablePoseDetection();
            message.enable = enable;
            pub_->publish(message);

            std::this_thread::sleep_for(1000ms);

            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<human_pose_interfaces::msg::EnablePoseDetection>::SharedPtr pub_;
};
