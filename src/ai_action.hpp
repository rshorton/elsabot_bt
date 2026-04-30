#pragma once

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "ai_session.hpp"
#include "ros_common.hpp"

class AIAction : public BT::StatefulActionNode {
 public:
  AIAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();
  void onHalted();

 private:
  BT::NodeStatus next_tool_call();
  void add_new_streaming_data(const std::string& delta);

  const std::size_t MIN_SENTENCE_LEN = 30;

  rclcpp::Node::SharedPtr node_;

#if defined(VLLM_GEMMA)  
  std::string model_{"cyankiwi/gemma-4-26B-A4B-it-AWQ-4bit"};
  std::string resource_{"/v1/chat/completions"};
  int max_context_size_{64000};
#else
  std::string resource_{"/completions"};
  int max_context_size_{4096};
#endif
  std::string auth_token_{"none"};

  std::string host_address_and_port_{"http://localhost:8000"};
  int timeout_ms_ = 8000;

  std::unique_ptr<AISession> ai_session_;

  std::string streaming_data_buffer_;     // Buffer for streamed results

  std::mutex new_sentence_mutex_;         // Protects new_sentence var
  std::string new_sentence_;              // New sentence ready to output

  nlohmann::json tool_calls_;             // Tool calls to make
  size_t tool_call_index_{0};             // Index of current tool call

  std::string tool_call_id_;              // ID of current tool call in progress
  std::string tool_call_name_;            // Name of current tool call in progress

  bool finish_on_next_update_{false};     // Finish running on next tick
  bool tool_call_wait_{false};            // Waiting on tool to complete
  std::chrono::time_point<std::chrono::steady_clock> tool_start_time_;
  float tool_call_timeout_sec_{30.0f};
  float last_tool_wait_report_{0.0f};
};
