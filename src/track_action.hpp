#pragma once

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/track.hpp"
#include <behaviortree_cpp_v3/action_node.h>

// Single for publishing the Track control state - shared by all TrackActionNode instances
class TrackActionROSNodeIf
{
public:
	TrackActionROSNodeIf(TrackActionROSNodeIf const&) = delete;
	TrackActionROSNodeIf& operator=(TrackActionROSNodeIf const&) = delete;

    static std::shared_ptr<TrackActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
        static std::shared_ptr<TrackActionROSNodeIf> s{new TrackActionROSNodeIf(node)};
        return s;
    }

    void setTrackState(std::string mode, std::string rate, bool detect_voice, bool turn_base)
    {
    	RCLCPP_INFO(node_->get_logger(), "Set track: mode= %s, rate= %s, detect_voice= %d, turn_base= %d",
    				mode.c_str(), rate.c_str(), detect_voice, turn_base);
    	auto msg = face_control_interfaces::msg::Track();
    	msg.mode = mode;
        msg.rate = rate;
        msg.voice_detect = detect_voice;
        msg.turn_base = turn_base;
        track_publisher_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }

private:
    TrackActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
    	track_publisher_ = node_->create_publisher<face_control_interfaces::msg::Track>("/head/track", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<face_control_interfaces::msg::Track>::SharedPtr track_publisher_;
};

class TrackAction : public BT::SyncActionNode
{
    public:
		TrackAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = TrackActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("mode"),
        			 BT::InputPort<std::string>("rate"),
					 BT::InputPort<bool>("detect_voice"),
        			 BT::InputPort<bool>("turn_base")};
        }

        virtual BT::NodeStatus tick() override
        {
        	std::string mode;
        	std::string rate;
        	bool detect_voice = false;
        	bool turn_base = false;
        	if (!getInput<std::string>("mode", mode)) {
    			throw BT::RuntimeError("missing mode");
    		}
        	if (!getInput<std::string>("rate", rate)) {
    			throw BT::RuntimeError("missing rate");
    		}
        	if (!getInput<bool>("turn_base", turn_base)) {
    			throw BT::RuntimeError("missing turn_base");
    		}

        	node_if_->setTrackState(mode, rate, detect_voice, turn_base);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<TrackActionROSNodeIf> node_if_;
};
