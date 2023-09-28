/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#pragma once

#include <stdio.h>
#include <sstream>
#include <string>
#include <limits>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "object_detection_msgs/msg/object_desc_array.hpp"

#include <behaviortree_cpp_v3/action_node.h>

typedef std::chrono::duration<float> float_seconds;

// Singleton for subscribing to object detection message - shared by all ObjectDetection node instances
class ObjectDetectionROSNodeIf
{
public:
	ObjectDetectionROSNodeIf(ObjectDetectionROSNodeIf const&) = delete;
	ObjectDetectionROSNodeIf& operator=(ObjectDetectionROSNodeIf const&) = delete;

    static std::shared_ptr<ObjectDetectionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<ObjectDetectionROSNodeIf> s{new ObjectDetectionROSNodeIf(node)};
        return s;
    }

    void update()
    {
    	rclcpp::spin_some(node_);
    }

    rclcpp::Node::SharedPtr node_;
    object_detection_msgs::msg::ObjectDescArray objArray_;
    bool detected_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;

private:
    ObjectDetectionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node),
    	detected_(false),
		last_time_(std::chrono::high_resolution_clock::now())
	{
        detected_obj_sub_ = node_->create_subscription<object_detection_msgs::msg::ObjectDescArray>(
            "/head/detected_objects",
			rclcpp::SystemDefaultsQoS(),
			std::bind(&ObjectDetectionROSNodeIf::detectCallback, this, std::placeholders::_1));
    }

    void detectCallback(object_detection_msgs::msg::ObjectDescArray::SharedPtr msg)
    {
    	objArray_ = *msg;
    	detected_ = true;
        last_time_ = std::chrono::high_resolution_clock::now();
    	//RCLCPP_INFO(node_->get_logger(), "Got detected objects");
    }

    rclcpp::Subscription<object_detection_msgs::msg::ObjectDescArray>::SharedPtr detected_obj_sub_;
};

class ObjectDetectionAction : public BT::SyncActionNode
{
    public:
	ObjectDetectionAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
			node_if_ = ObjectDetectionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
			return{
				BT::InputPort<std::string>("class"),
				BT::InputPort<float>("min_confidence"),
				BT::InputPort<int>("min_detect_count"),
				BT::OutputPort<std::string>("pose"),
				BT::OutputPort<double>("distance")};
        }

        virtual BT::NodeStatus tick() override
        {
        	auto now = std::chrono::high_resolution_clock::now();
        	auto elapsed = now - node_if_->last_time_;
        	node_if_->last_time_ = now;
        	auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

        	// Flush old data
        	if (seconds.count() > 1.0) {
        		node_if_->detected_ = false;
        	}

        	node_if_->update();

			std::string object_class;
			if (!getInput<std::string>("class", object_class)) {
				throw BT::RuntimeError("missing object_class");
			}

			float min_confidence;
			if (!getInput<float>("min_confidence", min_confidence)) {
				throw BT::RuntimeError("missing min_confidence");
			}

			RCLCPP_INFO(node_if_->node_->get_logger(), "Check for objects: class= [%s], detected= [%d]",
					object_class.c_str(),
					node_if_->detected_);

			if (!node_if_->detected_) {
				return BT::NodeStatus::FAILURE;
			}
			node_if_->detected_ = false;

			int closest = -1;
			double closest_dist = std::numeric_limits<double>::infinity();
			for (size_t i = 0; i < node_if_->objArray_.objects.size(); i++) {
				object_detection_msgs::msg::ObjectDesc &obj = node_if_->objArray_.objects[i];
				if (obj.track_status == "TRACKED" && obj.name == object_class) { // && obj.confidence >= min_confidence) {
					double sqdist = obj.position.point.x * obj.position.point.x + obj.position.point.y * obj.position.point.y;
					if (sqdist < closest_dist) {
						closest_dist = sqdist;
						closest = i;
					}
				}
			}

			if (closest != -1) {
				std::stringstream str;
				str << node_if_->objArray_.objects[closest].position.point.x << ","
					<< node_if_->objArray_.objects[closest].position.point.y << ",0.0"
					<< std::endl;
				setOutput("pose", str.str());
				closest_dist = sqrt(closest_dist);
				setOutput("distance", (double)closest_dist);

				cout << "x " << node_if_->objArray_.objects[closest].position.point.x << ", y "
					<< node_if_->objArray_.objects[closest].position.point.y << ", dist "
					<< closest_dist	<< ", conf "
					<< node_if_->objArray_.objects[closest].confidence
					<< std::endl;

				if (node_if_->objArray_.objects[closest].confidence >= min_confidence) {
					return BT::NodeStatus::SUCCESS;
				}
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
        std::shared_ptr<ObjectDetectionROSNodeIf> node_if_;
};
