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

#include "rclcpp/rclcpp.hpp"

#include "detection_processor_container.hpp"
#include "ros_common.hpp"

#include "detection_wait_for_object_detected_action.hpp"

BT::NodeStatus DetectionWaitForObjectDetectedAction::onStart() {
	if (!getInput<std::string>("det", detector_)) {
		throw BT::RuntimeError("missing det");
	}

	if (!getInput<std::string>("obj_class", obj_class_)) {
		throw BT::RuntimeError("missing obj_class");
	}

	if (!getInput<double>("timeout_sec", timeout_sec_)) {
		throw BT::RuntimeError("missing timeout_sec");
	}

	start_time_ = std::chrono::steady_clock::now();
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectionWaitForObjectDetectedAction::onRunning() {
	auto node = ROSCommon::GetInstance()->GetNode();

	auto now = std::chrono::steady_clock::now();
	auto elapsed_sec = std::chrono::duration_cast<std::chrono::duration<float>>(now - start_time_).count();
	if (elapsed_sec > timeout_sec_) {
		RCLCPP_INFO(node->get_logger(), "%s, timed-out", name().c_str());
		return BT::NodeStatus::FAILURE;    
	}

	ObjDetProcContainer *container = ObjDetProcContainer::GetInstance();
	if (!container) {
		return BT::NodeStatus::FAILURE;
	}

	std::shared_ptr<ObjDetProc> proc = container->GetProc(detector_);
	if (!proc) {
		RCLCPP_ERROR(node->get_logger(), "Object detection processor [%s] does not exist",
			detector_.c_str());
		return BT::NodeStatus::FAILURE;
	}

	if (proc->IsDetected(obj_class_)) {
		return BT::NodeStatus::SUCCESS;
	}
	return BT::NodeStatus::RUNNING;
}       

void DetectionWaitForObjectDetectedAction::onHalted() {
}
