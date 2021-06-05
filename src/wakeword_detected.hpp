#pragma once

#include <ratio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include "speech_action_interfaces/msg/wakeword.hpp"

using namespace std::chrono;

class WakeWordDetected : public BT::SyncActionNode
{
    public:
	WakeWordDetected(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("wake_word_det_node");

            ww_sub_ = node_->create_subscription<speech_action_interfaces::msg::Wakeword>(
              "speech_detect/wakeword",
              rclcpp::SystemDefaultsQoS(),
              std::bind(&WakeWordDetected::wwCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts()
        {
        	return {BT::InputPort<float>("since_sec")};
        }

        virtual BT::NodeStatus tick() override
        {
        	rclcpp::spin_some(node_);

        	if (ww.length()) {
				float since_sec = 4.0;
				getInput("since_sec", since_sec);

				double diff = (node_->now() - ww_time).seconds();

				auto ww_temp = ww;
				ww = "";
				std::cout << "Wakeword: since last: " << diff << ", since_sec: " << since_sec << std::endl;
				if (ww_temp.length() > 0 && diff < since_sec) {
					RCLCPP_INFO(node_->get_logger(), "Wakeword Detected: return success");
					return BT::NodeStatus::SUCCESS;
				}
        	}
            return BT::NodeStatus::FAILURE;
        }

        void wwCallback(speech_action_interfaces::msg::Wakeword::SharedPtr msg)
        {
        	ww = msg->word;
        	ww_time = msg->stamp;
        	RCLCPP_INFO(node_->get_logger(), "Wakeword Detected: %s", ww.c_str());
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<speech_action_interfaces::msg::Wakeword>::SharedPtr ww_sub_;
        std::string ww;
        rclcpp::Time ww_time;
};
