#include <string>
#include <iostream>
#include <fstream>
#include <ostream>
#include <sstream>
#include <json.hpp>

#include "ros_common.hpp"
#include "tool_call_data.hpp"

ToolCallData::ToolCallData()
{
	tool_descriptions_ = nlohmann::json::array();
}

void ToolCallData::declare_tool(const std::string &tool_desc_json)
{
	auto logger = ROSCommon::GetInstance()->GetNode()->get_logger(); 
	try {
		tool_descriptions_.push_back(nlohmann::json::parse(tool_desc_json));
		RCLCPP_INFO(logger, "declare_tool, descriptions: %s", get_available_tools_json().c_str());
	} catch (nlohmann::json::parse_error& ex) {
        RCLCPP_ERROR(logger, "Error parsing tool description: %s at: %ld", tool_desc_json.c_str(), ex.byte);
	}		
}

std::string ToolCallData::get_available_tools_json()
{
	return tool_descriptions_.dump();
}

