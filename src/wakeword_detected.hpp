#pragma once

#include <ratio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <behaviortree_cpp_v3/action_node.h>

using namespace std::chrono;

class WakeWordDetected : public BT::SyncActionNode
{
    public:
	WakeWordDetected(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("wake_word_det_node");

            ww_sub_ = node_->create_subscription<std_msgs::msg::String>(
              "speech_detect/wakeword",
              rclcpp::SystemDefaultsQoS(),
              std::bind(&WakeWordDetected::wwCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts()
        {
        	return {BT::InputPort<std::string>("since_ms")};

        }

        virtual BT::NodeStatus tick() override
        {
        	rclcpp::spin_some(node_);

        	if (ww.length()) {
				unsigned since_ms = 1000;
				getInput("delay_msec", since_ms);

				steady_clock::time_point now = steady_clock::now();
				auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - ww_time);

				auto ww_temp = ww;
				ww = "";
				std::cout << "Wakeword: since last: " << ms.count() << ", since_ms: " << since_ms << std::endl;
				if (ww_temp.length() > 0 && ms.count() < since_ms) {
					RCLCPP_INFO(node_->get_logger(), "Wakeword Detected: return success");
					return BT::NodeStatus::SUCCESS;
				}
        	}
            return BT::NodeStatus::FAILURE;
        }

        void wwCallback(std_msgs::msg::String::SharedPtr msg)
        {
        	ww = msg->data;
        	ww_time = steady_clock::now();
        	RCLCPP_INFO(node_->get_logger(), "Wakeword Detected: %s", ww.c_str());
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ww_sub_;
        std::string ww;
        steady_clock::time_point ww_time;
};
