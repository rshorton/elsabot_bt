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

#include <string>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "detection_processor_container.hpp"
#include "ros_common.hpp"

#include "detection_get_objects_action.hpp"

using json = nlohmann::json;

BT::NodeStatus DetectionGetObjectsAction::tick()
{
	auto node = ROSCommon::GetInstance()->GetNode();

	std::string det;
	if (!getInput<std::string>("det", det)) {
		throw BT::RuntimeError("missing det");
	}

	std::string coord_frame;
	if (!getInput<std::string>("coord_frame", coord_frame)) {
		throw BT::RuntimeError("missing coord_frame");
	}

	ObjDetProcContainer *container = ObjDetProcContainer::GetInstance();
	if (!container) {
		return BT::NodeStatus::FAILURE;
	}

	std::shared_ptr<ObjDetProc> proc = container->GetProc(det);
	if (!proc) {
		RCLCPP_ERROR(node->get_logger(), "Object detection processor [%s] does not exist",
			det.c_str());
		return BT::NodeStatus::FAILURE;
	}

	std::vector<ObjDetProc::Detection> detections;

	if (!proc->GetObjects(coord_frame, detections)) {
		RCLCPP_ERROR(node->get_logger(), "Failed to get detected objects");
		return BT::NodeStatus::FAILURE;
	}

	json objs = json::array();
	for (const auto &obj: detections) {
		objs.push_back({{"id", obj.id},
						{"class", obj.obj_class},
						{"x", obj.pos.x},
						{"y", obj.pos.y},
						{"z", obj.pos.z},
						{"relative_yaw", obj.yaw},
						{"distance", obj.dist}});
	}

	auto objs_json = objs.dump();
	RCLCPP_INFO(node->get_logger(), "%s, Objs: %s", name().c_str(), objs_json.c_str());
	setOutput("objects", objs_json);
	return BT::NodeStatus::SUCCESS;
}
