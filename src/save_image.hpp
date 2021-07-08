#pragma once

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/save_image.hpp"
#include <behaviortree_cpp_v3/action_node.h>

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
    	auto message = face_control_interfaces::msg::SaveImage();
        message.filepath = directory;
        save_image_publisher_->publish(message);
    }

private:
    SaveImageActionROSNodeIf(rclcpp::Node::SharedPtr node):
		 node_(node)
	{
        save_image_publisher_ = node_->create_publisher<face_control_interfaces::msg::SaveImage>("/head/save_image", 2);
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<face_control_interfaces::msg::SaveImage>::SharedPtr save_image_publisher_;
};

class SaveImageAction : public BT::SyncActionNode
{
    public:
		SaveImageAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
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
