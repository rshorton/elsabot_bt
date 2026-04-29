/*
Copyright 2026 Scott Horton

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
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

#include <behaviortree_cpp/action_node.h>

#include "base64.hpp"

// Singleton for subscribing to camera topic
class GetCameraFrameActionROSNodeIf
{
public:
	GetCameraFrameActionROSNodeIf(GetCameraFrameActionROSNodeIf const&) = delete;
	GetCameraFrameActionROSNodeIf& operator=(GetCameraFrameActionROSNodeIf const&) = delete;

    static std::shared_ptr<GetCameraFrameActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<GetCameraFrameActionROSNodeIf> s{new GetCameraFrameActionROSNodeIf(node)};
        return s;
    }

    void update()
    {
    	rclcpp::spin_some(node_);
    }

    float time_since_sub() {
   		auto now = std::chrono::steady_clock::now();
		return std::chrono::duration_cast<std::chrono::duration<float>>(now - sub_time_).count();
    }

    void reset()
    {
        base64_image_.clear();
        sub_time_ = std::chrono::steady_clock::now();
    }

    void subscribe() {
        reset();
        if (sub_) {
            return;
        }

        sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>("color/image/compressed", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                RCLCPP_INFO(node_->get_logger(), "Received image. Format: %s, Size: %zu bytes", msg->format.c_str(), msg->data.size());
                base64_image_ = base64_encode(msg->data);
                RCLCPP_INFO(node_->get_logger(), "Base64 string size: %zu", base64_image_.size());
            });
        RCLCPP_INFO(node_->get_logger(), "Subscribed to image topic");
    }

    void unsubscribe() {
        sub_ = nullptr;
    }

    rclcpp::Node::SharedPtr node_;
    std::string base64_image_;

private:
    GetCameraFrameActionROSNodeIf(rclcpp::Node::SharedPtr node):
    	node_(node)
	{
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    std::chrono::time_point<std::chrono::steady_clock> sub_time_;
};

class GetCameraFrameAction : public BT::StatefulActionNode
{
    public:
	GetCameraFrameAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::StatefulActionNode(name, config)
    {
        node_if_ = GetCameraFrameActionROSNodeIf::instance(node);
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<std::string>("base64_image")};
    }

    BT::NodeStatus onStart() {
        node_if_->subscribe();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        node_if_->update();

        auto result = BT::NodeStatus::FAILURE;
        std::string result_json;

        if (node_if_->base64_image_.empty()) {
            if (node_if_->time_since_sub() < timeout_) {
                return BT::NodeStatus::RUNNING;
            }
            RCLCPP_ERROR(node_if_->node_->get_logger(), "Error, GetCameraFrameAction failed to get image");
        } else {
            // fix - check format and build prefix with type
            auto image = "data:image/jpeg;base64," + node_if_->base64_image_;
            setOutput("base64_image", image);
            result = BT::NodeStatus::SUCCESS;
        }
        if (result != BT::NodeStatus::RUNNING) {
            node_if_->unsubscribe();
        }
        return result; 
    }

    void onHalted() {
        node_if_->unsubscribe();
    }

    private:
    std::shared_ptr<GetCameraFrameActionROSNodeIf> node_if_;
    float timeout_{4.0f};
};
