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

#ifndef _DETECTION_PROCESSOR_CONTAINER_HPP_
#define _DETECTION_PROCESSOR_CONTAINER_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "detection_processor.hpp"

class ObjDetProcContainer
{
private:

public:

	static ObjDetProcContainer* Create(rclcpp::Node::SharedPtr node)
	{
		if (!container_) {
			container_ = new ObjDetProcContainer(node);
		}			
		return container_;
	};

	static ObjDetProcContainer* GetInstance()
	{
		return container_;
	};

	std::shared_ptr<ObjDetProc> CreateProc(const std::string &name, const std::string &topic);
	bool DestroyProc(const std::string &name);

	std::shared_ptr<ObjDetProc> GetProc(const std::string &name);

private:
	ObjDetProcContainer(rclcpp::Node::SharedPtr node): node_(node) {};
	~ObjDetProcContainer() {};

private:
	static ObjDetProcContainer *container_;

	rclcpp::Node::SharedPtr node_;
	std::unordered_map<std::string, std::shared_ptr<ObjDetProc>> detector_map_;
};

#endif //_DETECTION_PROCESSOR_CONTAINER_HPP_
