/*
Copyright 2023 Scott Horton

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

#include <iomanip>
#include <iostream>
#include <fstream>
#include <regex>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "robot_localization/srv/from_ll.hpp"

class LLCoordToLocalUTM : public BT::SyncActionNode
{
    using pose_list = std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>;

public:
    LLCoordToLocalUTM(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
    	return {
            BT::InputPort<std::string>("ll_poses"),         // Provide list of 2D poses, or
            BT::InputPort<std::string>("ll_kml_path"),      // the path to a path KML file with a list of coordinates.
                                                            // You can use Google Earth Pro to create.
            BT::OutputPort<pose_list>("local_utm_coord_poses")
        };
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("ll_to_local_utm");

        std::vector<Pose2D> poses;
        std::string ll_kml_path;
        std::string ll_poses;
        
        if (!getInput<std::string>("ll_poses", ll_poses)) {
            if (!getInput<std::string>("ll_kml_path", ll_kml_path)) {
                throw BT::RuntimeError("LLCoordToLocalUTM requires ll_poses or ll_krl_path");
            }
        }            

        if (ll_poses.size()) {
            RCLCPP_INFO(node_->get_logger(), "Using pose list");
            // lat,long,yaw_deg poses separated by semicolon
            poses = convertPose2DListFromString(ll_poses, ';');

        } else {
            RCLCPP_INFO(node_->get_logger(), "Using kml pose list");
            read_poses_from_kml(ll_kml_path, poses);
        }

        if (!poses.size()) {
            RCLCPP_ERROR(node_->get_logger(), "LLCoordToLocalUTM failed to prepare pose list");
            return BT::NodeStatus::FAILURE;
        }

        auto client = node_->create_client<robot_localization::srv::FromLL>("fromLL");

        // if no server is present, fail after N seconds
        if (!client->wait_for_service(std::chrono::seconds(20))) {
            RCLCPP_ERROR(node_->get_logger(), "LL to UTM action service not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        auto ok = true;
        auto utm_poses = std::make_shared<std::vector<geometry_msgs::msg::PoseStamped>>();

        for (const auto &pose: poses) {
            auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
            request->ll_point.latitude = pose.x;
            request->ll_point.longitude = pose.y;
            request->ll_point.altitude = 0.0;

            RCLCPP_INFO(node_->get_logger(), "Sending LL to UTM request for point: %f, %f",
                pose.x, pose.y);

            auto result = client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "completed");

            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to convert LL coord");
                ok = false;
                break;
            }

            auto map_point = result.get()->map_point;

            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = "map";
            p.header.stamp = node_->now();
            p.pose.position.x = map_point.x;
            p.pose.position.y = map_point.y;
            p.pose.position.z = map_point.z;

            tf2::Quaternion tf_quat;
            tf_quat.setRPY(0.0, 0.0, pose.yaw);
            p.pose.orientation = tf2::toMsg(tf_quat);

            RCLCPP_INFO(node_->get_logger(), "local utm: %f, %f ", map_point.x, map_point.y);

            utm_poses->push_back(p);
        }

        if (!ok) {
            return BT::NodeStatus::FAILURE;
        }

		setOutput("local_utm_coord_poses", utm_poses);
        return BT::NodeStatus::SUCCESS;
    }

    // Parse the coordinates from a KML file created by exporting a Path from Google Earth Pro
    bool read_poses_from_kml(const std::string &ll_kml_path, std::vector<Pose2D> &poses)
    {
        std::ifstream ifs(ll_kml_path);
        std::stringstream ss;
        ss << ifs.rdbuf();

        const std::string &s = ss.str();

        if (!s.size()) {
            RCLCPP_INFO(node_->get_logger(), "kml file is empty or not found");
            return false;
        }

        std::regex rgx(".*<coordinates>[\\n\\r\\s]*(.*)[\\n\\r\\s]*</coordinates>");
        std::smatch match;

        RCLCPP_DEBUG(node_->get_logger(), "kml %s", s.c_str());

        if (std::regex_search(s, match, rgx)) {
            std::string coords = match.str(1);
            RCLCPP_INFO(node_->get_logger(), "Coords %s", coords.c_str());

            // Parse the lat,long,altitude values into a 2D pose object.
            // Since the altitude is not used and mapped to the yaw
            // member, just ensure the yaw is defaulted after parsing.
            poses = convertPose2DListFromString(coords, ' ', true);
            for (auto &pose: poses) {

                pose.yaw = 0.0;
            }

            RCLCPP_INFO(node_->get_logger(), "Parsed %ld coords from KML", poses.size());
            return true;
        }
        return false;
    }

private:
    rclcpp::Node::SharedPtr node_;
};
