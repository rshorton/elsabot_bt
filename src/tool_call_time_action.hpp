#include <nlohmann/json.hpp>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "behaviortree_cpp/bt_factory.h"
#include "ros_common.hpp"
#include "tool_call_data.hpp"

#undef TEST_HOLDING_OFF_RESULT   // Simulates a long running action

using namespace std::chrono_literals;

// Test tool call that gets the current date and time

class ToolCallTimeAction : public BT::StatefulActionNode {
 public:
  ToolCallTimeAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {
    node_ = ROSCommon::GetInstance()->GetNode();        
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("command"),
            BT::InputPort<std::string>("args_json"),
            BT::OutputPort<std::string>("result_json")};
  }

  BT::NodeStatus onStart() {
    std::string command;
    getInput<std::string>("command", command);

    // Report tool description on init
    if (command == "init" || command.empty()) {
      ToolCallData& tc_data = ToolCallData::getInstance();
      tc_data.declare_tool(tool_desc_);
      return BT::NodeStatus::SUCCESS;
    }

#if defined(TEST_HOLDING_OFF_RESULT)
    start_ = std::chrono::high_resolution_clock::now();
    return BT::NodeStatus::RUNNING;
#else
    get_and_output_result();
    return BT::NodeStatus::SUCCESS;
#endif    
  }

  BT::NodeStatus onRunning() {
#if defined(TEST_HOLDING_OFF_RESULT)
		auto elapsed = std::chrono::high_resolution_clock::now() - start_;
		auto seconds = std::chrono::duration_cast<std::chrono::duration<float>>(elapsed);
		if (seconds.count() < 3.0) {
      return BT::NodeStatus::RUNNING;
    }
    get_and_output_result();
#endif    
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() {
  }

 private:
  void get_and_output_result() {
    std::string args_json;
    getInput<std::string>("args_json", args_json);

    nlohmann::json result_obj = {{"result", get_local_date_and_time()}};
    auto result = result_obj.dump();

    RCLCPP_INFO(node_->get_logger(), "Tool call, time, result: %s", result.c_str());

    setOutput("result_json", result);
  }

  std::string get_local_date_and_time() {
    auto now = std::chrono::system_clock::now();

    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm local_tm; 
    localtime_r(&now_time_t, &local_tm);

    std::stringstream s;
    s << std::put_time(&local_tm, "%Y-%m-%d")
      << " "
      << std::put_time(&local_tm, "%H:%M:%S");

    return s.str();
  }

  rclcpp::Node::SharedPtr node_;

  std::string tool_desc_ = R"(
    {
      "type": "function",
      "function": {
        "name": "get_local_date_and_time",
        "description": "Gets the local date and time time in the form:  Y-M-D H:M:S.  Always re-run this tool when the date or time is needed."
      }
    })";

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};
