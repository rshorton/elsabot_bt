#ifndef _TOOL_CALL_DATA_HPP_
#define _TOOL_CALL_DATA_HPP_

#include <string>
#include <json.hpp>

// Class for aggregating the descriptions of the available tool calls.  This list is
// consumed by the ai_action node when creating a model request where tool calls are
// allowed.

class ToolCallData
{
    public:
        static ToolCallData& getInstance()
        {
            static ToolCallData instance;
            return instance;
        }
    private:
        ToolCallData();
    public:
        ToolCallData(ToolCallData const&) = delete;
        void operator=(ToolCallData const&) = delete;

        void declare_tool(const std::string &tool_desc_json);
        std::string get_available_tools_json();

    private:
        nlohmann::json tool_descriptions_;
};

#endif // _TOOL_CALL_DATA_HPP_
