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
		throw std::runtime_error("Invalid tool description");
	}		
}

std::string ToolCallData::get_available_tools_json()
{
	return tool_descriptions_.dump();
}

void ToolCallData::set_subtree_runner(const std::string &tool_name, const std::string &subtree_name)
{
	sub_tree_to_tool_map_[tool_name] = subtree_name;
}

bool ToolCallData::get_subtree_runner(const std::string &tool_name, std::string &subtree_name) const
{
	auto it = sub_tree_to_tool_map_.find(tool_name);
	if (it != sub_tree_to_tool_map_.end()) {
		subtree_name = it->second;
		return true;
	}
	return false;
}
