#pragma once

#include <stdio.h>
#include <chrono>
#include <fstream>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "human_pose_interfaces/msg/detected_pose.hpp"

#include <behaviortree_cpp_v3/action_node.h>

using namespace std::chrono_literals;
using std::map;

// Singleton for subscribing to human pose detection message - shared by all HumanPoseDetect node instances
class HumanPoseDetectROSNodeIf
{
public:
	HumanPoseDetectROSNodeIf(HumanPoseDetectROSNodeIf const&) = delete;
	HumanPoseDetectROSNodeIf& operator=(HumanPoseDetectROSNodeIf const&) = delete;

    static std::shared_ptr<HumanPoseDetectROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<HumanPoseDetectROSNodeIf> s{new HumanPoseDetectROSNodeIf(node)};
        return s;
    }

    void update()
    {
    	rclcpp::spin_some(node_);
    }

    rclcpp::Node::SharedPtr node_;
    bool detected_;
    int num_points_;
    std::string cur_pose_left_;
    std::string cur_pose_right_;

private:
    HumanPoseDetectROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
    	detected_(false),
		num_points_(0)
	{
        detected_pose_sub_ = node_->create_subscription<human_pose_interfaces::msg::DetectedPose>(
            "/head/detected_pose",
			rclcpp::SystemDefaultsQoS(),
			std::bind(&HumanPoseDetectROSNodeIf::poseCallback, this, std::placeholders::_1));
    }

    void poseCallback(human_pose_interfaces::msg::DetectedPose::SharedPtr msg)
    {
    	cur_pose_left_ = msg->left;
    	cur_pose_right_ = msg->right;
    	detected_ = msg->detected;
    	num_points_ = msg->num_points;
		RCLCPP_INFO(node_->get_logger(), "Got pose callback [%s], [%s]", cur_pose_left_.c_str(), cur_pose_right_.c_str());
    }

    rclcpp::Subscription<human_pose_interfaces::msg::DetectedPose>::SharedPtr detected_pose_sub_;
};

class HumanPoseDetect : public BT::SyncActionNode
{
    public:
		HumanPoseDetect(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
				: BT::SyncActionNode(name, config)
		{
			node_if_ = HumanPoseDetectROSNodeIf::instance(node);
			speech_strings_ = createPoseToSpeechMap();
		}

		static BT::PortsList providedPorts()
		{
			return{
				BT::InputPort<std::string>("expected_pose_left"),
				BT::InputPort<std::string>("expected_pose_right"),
				BT::InputPort<std::string>("pose_lr_check"),
				BT::InputPort<int>("min_points"),
				BT::OutputPort<std::string>("detected_person"),
				BT::OutputPort<std::string>("pose_left"),
				BT::OutputPort<std::string>("pose_right"),
				BT::OutputPort<std::string>("pose_left_speech"),
				BT::OutputPort<std::string>("pose_right_speech")};
		}

		const std::string getSpeechText(std::string pose)
		{
			return speech_strings_[pose];
		}

		map<std::string, std::string> createPoseToSpeechMap()
		{
			map<std::string, std::string> m  = {
				{"OnHip", "hand touching hip"},
				{"ArmOut", "arm Out"},
				{"ArmToSide", "arm To Side"},
				{"TouchingShoulder", "hand touching Shoulder"},
				{"TouchingStomach", "hand touching Stomach"},
				{"Abovehead", "hand above head"},
				{"OnHead", "hand on head"},
				{"TouchingNeck", "hand touching neck"},
				{"Unknown", "None"}
			};
			return m;
		}

		virtual BT::NodeStatus tick() override
		{
			node_if_->update();

			setOutput("pose_left", node_if_->cur_pose_left_);
			setOutput("pose_left", node_if_->cur_pose_right_);
			setOutput("detected_person", node_if_->detected_? "yes": "no");

			RCLCPP_INFO(node_if_->node_->get_logger(), "[%s], [%s]",
						getSpeechText(node_if_->cur_pose_left_).c_str(),
						getSpeechText(node_if_->cur_pose_right_).c_str());

			setOutput("pose_left_speech", getSpeechText(node_if_->cur_pose_left_));
			setOutput("pose_right_speech", getSpeechText(node_if_->cur_pose_right_));

			int min_points = 2;
			getInput<int>("min_points", min_points);

			std::string expected_left;
			if (!getInput<std::string>("expected_pose_left", expected_left)) {
				throw BT::RuntimeError("missing expected pose_left");
			}

			std::string expected_right;
			if (!getInput<std::string>("expected_pose_right", expected_right)) {
				throw BT::RuntimeError("missing expected pose_right");
			}

			std::string pose_lr_check = "any";
			getInput<std::string>("pose_lr_check", pose_lr_check);

			RCLCPP_INFO(node_if_->node_->get_logger(), "Check for poses: person_det= [%d], Compare: [%s],  L [%s], R [%s], current L [%s], R[%s]",
					node_if_->detected_,
					pose_lr_check.c_str(),
					expected_left.c_str(),
					expected_right.c_str(),
					node_if_->cur_pose_left_.c_str(),
					node_if_->cur_pose_right_.c_str());

			// Return success if both of the expected left/right poses are seen.  Either or both
			// can be required.  If neither l/r poses are specified, then success if a person
			// is detected at all.
			if (node_if_->detected_ && node_if_->num_points_ >= min_points) {
				if (pose_lr_check.compare("presence") == 0) {
					return BT::NodeStatus::SUCCESS;
				}

				bool okL = false;
				bool okR = false;

				if (expected_left.length() != 0 && expected_left.compare(node_if_->cur_pose_left_) == 0) {
					okL = true;
				}
				if (expected_right.length() != 0 && expected_right.compare(node_if_->cur_pose_right_) == 0) {
					okR = true;
				}

				if ((pose_lr_check.compare("left") == 0 && okL) ||
					(pose_lr_check.compare("right") == 0 && okR) ||
					(pose_lr_check.compare("any") == 0 && (okL || okR)) ||
					(pose_lr_check.compare("both") == 0 && okL && okR)) {
					return BT::NodeStatus::SUCCESS;
				}
			}
			return BT::NodeStatus::FAILURE;
		}

    private:
        std::shared_ptr<HumanPoseDetectROSNodeIf> node_if_;
        map<std::string, std::string> speech_strings_;
};
