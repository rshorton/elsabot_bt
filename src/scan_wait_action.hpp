#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "face_control_interfaces/msg/scan_status.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class ScanWaitAction: public CoroActionNode
{
  public:
	ScanWaitAction(const std::string& name):
        CoroActionNode(name, {})
    {
        node_ = rclcpp::Node::make_shared("scan_wait_action");

        sub_ = node_->create_subscription<face_control_interfaces::msg::ScanStatus>(
            "/head/scan_status",
			rclcpp::SystemDefaultsQoS(),
			std::bind(&ScanWaitAction::scanStatusCallback, this, std::placeholders::_1));
    }

    void scanStatusCallback(face_control_interfaces::msg::ScanStatus::SharedPtr msg)
    {
    	msg_ = *msg;
    	received_ = true;
		//RCLCPP_INFO(node_->get_logger(), "Got scan status msg");
    }

  private:
    NodeStatus tick() override
    {
    	received_ = false;
    	face_control_interfaces::msg::ScanStatus start_msg;

    	int state = 0;

        while (true) {
        	rclcpp::spin_some(node_);

        	if (state == 0) {
        		if (received_) {
        			received_ = false;
        			start_msg = msg_;
        			RCLCPP_INFO(node_->get_logger(), "lcnt= %d, rcnt= %d",
        					start_msg.at_left_count, start_msg.at_right_count);
        			state = 1;
        		}
        	} else if (state == 1) {
        		if (received_) {
        			received_ = false;
        			RCLCPP_INFO(node_->get_logger(), "lcnt= %d, rcnt= %d",
        					start_msg.at_left_count, start_msg.at_right_count);
        			if (start_msg.at_left_count != msg_.at_left_count &&
       					start_msg.at_right_count != msg_.at_right_count) {
						return NodeStatus::SUCCESS;
        			}
        		}
        	}
            // set status to RUNNING and "pause/sleep"
            // If halt() is called, we will not resume execution (stack destroyed)
            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << name() <<": Halted." << std::endl;
        // Do not forget to call this at the end.
        CoroActionNode::halt();
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<face_control_interfaces::msg::ScanStatus>::SharedPtr sub_;

    face_control_interfaces::msg::ScanStatus msg_;
    bool received_;
};
