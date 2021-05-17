#pragma once

#include <stdio.h>
#include <sstream>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "object_detection_msgs/msg/object_desc_array.hpp"

#include <behaviortree_cpp_v3/action_node.h>

class ObjectDetectionAction : public BT::SyncActionNode
{
    public:
	ObjectDetectionAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config),
			  detected_(false)
        {
            node_ = rclcpp::Node::make_shared("object_detection_node");

            detected_pose__sub_ = node_->create_subscription<object_detection_msgs::msg::ObjectDescArray>(
                "/head/detected_objects",
				rclcpp::SystemDefaultsQoS(),
				std::bind(&ObjectDetectionAction::detectCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts()
        {
			return{
				BT::InputPort<std::string>("class"),
				BT::InputPort<float>("min_confidence"),
				BT::OutputPort<std::string>("pose")};
        }

        void detectCallback(object_detection_msgs::msg::ObjectDescArray::SharedPtr msg)
        {
        	objArray_ = *msg;
        	detected_ = true;
			RCLCPP_INFO(node_->get_logger(), "Got detected objects");
        }

        virtual BT::NodeStatus tick() override
        {
        	rclcpp::spin_some(node_);

			std::string object_class;
			if (!getInput<std::string>("class", object_class)) {
				throw BT::RuntimeError("missing object_class");
			}

			float min_confidence;
			if (!getInput<float>("min_confidence", min_confidence)) {
				throw BT::RuntimeError("missing min_confidence");
			}

			RCLCPP_INFO(node_->get_logger(), "Check for objects: class= [%s], detected= [%d]",
					object_class.c_str(),
					detected_);

			int closest_person = -1;
			float closest_dist = std::numeric_limits<float>::infinity();
			for (size_t i = 0; i < objArray_.objects.size(); i++) {
				object_detection_msgs::msg::ObjectDesc &obj = objArray_.objects[i];
				if (obj.name == object_class && obj.confidence >= min_confidence) {
					float sqdist = obj.x * obj.x + obj.y * obj.y;
					if (sqdist < closest_dist) {
						closest_dist = sqdist;
						closest_person = i;
					}
				}
			}

			if (closest_person != -1) {
				std::stringstream str;
				str << objArray_.objects[closest_person].x << ","
					<< objArray_.objects[closest_person].x << ",0.0"
					<< std::endl;
				setOutput("pose", str.str());
				return BT::NodeStatus::SUCCESS;
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<object_detection_msgs::msg::ObjectDescArray>::SharedPtr detected_pose__sub_;

        object_detection_msgs::msg::ObjectDescArray objArray_;
        bool detected_;
};
