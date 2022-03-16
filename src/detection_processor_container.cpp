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

/*
This class is used to create and contain Object Detection Processor
instances.  A tree node can request the creation and other nodes can
query the status of the detector at any time. 
*/

#include <iostream>
#include <string>
#include <map>
#include <chrono>

#include "detection_processor_container.hpp"

ObjDetProcContainer *ObjDetProcContainer::container_ = nullptr;

std::shared_ptr<ObjDetProc> ObjDetProcContainer::CreateProc(const std::string &name, const std::string &topic)
{
	std::shared_ptr<ObjDetProc> det;

	auto it = detector_map_.find(name);
	if (it != detector_map_.end()) {
		return it->second;
	}

	det = std::make_shared<ObjDetProc>(node_, name, topic);
	if (det) {
		detector_map_[name] = det;
	}
	RCLCPP_INFO(node_->get_logger(), "%s detection processor [%s] for topic [%s]",
			det? "Created ": "Failed to create ",
			name.c_str(), topic.c_str());
	return det;
}

bool ObjDetProcContainer::DestroyProc(const std::string &name)
{
	return detector_map_.erase(name) != 0;
}

std::shared_ptr<ObjDetProc> ObjDetProcContainer::GetProc(const std::string &name)
{
	std::shared_ptr<ObjDetProc> det;
	auto it = detector_map_.find(name);
	if (it != detector_map_.end()) {
		det = it->second;
	}
	return det;
}
