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

#include "rclcpp/rclcpp.hpp"
#include "robot_head_interfaces/msg/save_image.hpp"
#include <behaviortree_cpp/action_node.h>

// Singleton for publishing the save image message - shared by all SaveImageAction nodes instances
class SaveImageActionROSNodeIf
{
public:
	SaveImageActionROSNodeIf(SaveImageActionROSNodeIf const&) = delete;
	SaveImageActionROSNodeIf& operator=(SaveImageActionROSNodeIf const&) = delete;

    static std::shared_ptr<SaveImageActionROSNodeIf> instance(rclcpp::Node::SharedPtr node)
    {
    	static std::shared_ptr<SaveImageActionROSNodeIf> s{new SaveImageActionROSNodeIf(node)};
        return s;
    }

    void saveImage(std::string directory, std::string filename)
    {
    	directory += "/";
    	directory += filename;
    	RCLCPP_INFO(node_->get_logger(), "Save image: %s", directory.c_str());
    	auto message = robot_head_interfaces::msg::SaveImage();
        message.filepath = directory;
        save_image_publisher_->publish(message);
    }

private:
    SaveImageActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        save_image_publisher_ = node_->create_publisher<robot_head_interfaces::msg::SaveImage>("/head/save_image", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_head_interfaces::msg::SaveImage>::SharedPtr save_image_publisher_;
};

class SaveImageAction : public BT::SyncActionNode
{
    public:
		SaveImageAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
            : BT::SyncActionNode(name, config)
        {
            node_if_ = SaveImageActionROSNodeIf::instance(node);
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("filename"), BT::InputPort<std::string>("directory") };
        }

        virtual BT::NodeStatus tick() override
        {
        	std::string filename;
        	std::string directory;
        	if (!getInput<std::string>("filename", filename)) {
    			throw BT::RuntimeError("missing filename");
    		}
        	if (!getInput<std::string>("directory", directory)) {
    			throw BT::RuntimeError("missing directory");
    		}
        	node_if_->saveImage(directory, filename);
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<SaveImageActionROSNodeIf> node_if_;
};
