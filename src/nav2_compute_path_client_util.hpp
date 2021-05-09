#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

bool GetPathToPose(rclcpp::Node::SharedPtr node, nav_msgs::msg::Path &path, double &len, double x, double y);
