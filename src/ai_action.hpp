#pragma once

#include <string>
#include <chrono>

#include "behaviortree_cpp/bt_factory.h"
#include "ros_common.hpp"

class AISession;

class AIAction : public BT::StatefulActionNode {
private:
  const std::string DEFAULT_MODEL =  "cyankiwi/gemma-4-26B-A4B-it-AWQ-4bit";
  const size_t DEFAULT_CONTEXT_SIZE_LIMIT = 64000;
  const std::string DEFAULT_IP_ADDR_AND_PORT = "http://localhost:8000";

public:
  AIAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();
  void onHalted();

private:
  BT::NodeStatus next_tool_call();
  void remove_silent_chars(std::string &str);
  void add_new_streaming_data(const std::string& delta);
  void process_full_response(const std::string& response);
  BT::NodeStatus update_waiting_on_response(bool abort);
  BT::NodeStatus update_waiting_on_tool_call(bool abort);

  const std::size_t MIN_SENTENCE_LEN = 30;

  rclcpp::Node::SharedPtr node_;

  //std::string model_{"cyankiwi/gemma-4-31B-it-AWQ-4bit"};
  std::string model_{DEFAULT_MODEL};
  std::string resource_{"/v1/chat/completions"};
  size_t max_context_size_{DEFAULT_CONTEXT_SIZE_LIMIT};
  std::string auth_token_{"none"};
  std::string host_address_and_port_{DEFAULT_IP_ADDR_AND_PORT};
  int timeout_ms_ = 8000;

  bool parallel_tool_call_support_{true};

  std::unique_ptr<AISession> ai_session_;
  bool stream_{false};                    // True to have LLM output streamed vs receive one full response

  std::string tools_json_;                // Current tools available to the model

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
