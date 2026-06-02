#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

#include "behaviortree_cpp/bt_factory.h"

#include "ai_session.hpp"
#include "ros_common.hpp"
#include "tool_call_data.hpp"

#include "ai_action.hpp"

AIAction::AIAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {
  node_ = ROSCommon::GetInstance()->GetNode();        
}

BT::PortsList AIAction::providedPorts() {
  return {BT::InputPort<std::string>("model"),
          BT::InputPort<size_t>("context_size_limit"),
          BT::InputPort<std::string>("system_prompt"),
          BT::InputPort<bool>("abort"),
          BT::InputPort<bool>("new_session"),
          BT::InputPort<std::string>("prompt"),
          BT::InputPort<bool>("streaming"),
          BT::InputPort<bool>("enable_reasoning"),
          BT::InputPort<float>("tool_call_timeout_sec"),
          BT::BidirectionalPort<std::string>("tool_call_result"),
          BT::BidirectionalPort<std::string>("tool_call_results"),
          BT::OutputPort<std::string>("result"),
          BT::OutputPort<std::string>("tool_call_name"),
          BT::OutputPort<std::string>("tool_call_args"),
          BT::OutputPort<std::string>("tool_calls")
        };
}

BT::NodeStatus AIAction::onStart() {
  auto data_callback = [&](const std::string &data) {
    add_new_streaming_data(data);
  };

  bool new_session = false;
  getInput<bool>("new_session", new_session);

  // Optional
  getInput<std::string>("model", model_);
  getInput<size_t>("context_size_limit", max_context_size_);

  if (new_session || !ai_session_) {
    std::string system_prompt = "You are a helpful assistant.";
    getInput<std::string>("system_prompt", system_prompt);

    // Note that timeout is set to 0 to avoid prematuring timing-out on streamed response.
    // Lower level handling uses a transfer based timeout that times-out if the rate is too low
    // for too long.
    ai_session_ = std::make_unique<AISession>(model_, max_context_size_, auth_token_,
                                              host_address_and_port_, resource_,
                                              0, system_prompt, node_->get_logger(),
                                              data_callback);
    if (!ai_session_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create an AI session object");
      return BT::NodeStatus::FAILURE;
    }                                                  
  }
 
  std::string prompt;
  if (!getInput<std::string>("prompt", prompt)) {
    throw BT::RuntimeError("missing prompt");
  }
  
  // Optional
  stream_ = false;
  getInput<bool>("streaming", stream_);

  // Optional
  bool enable_reasoning = false;
  getInput<bool>("enable_reasoning", enable_reasoning);

  finish_on_next_update_ = false;
  new_sentence_ = std::string();
  tool_call_wait_ = false;

  getInput<float>("tool_call_timeout_sec", tool_call_timeout_sec_);

  setOutput("tool_call_name", "");
  setOutput("tool_call_result", "");
  setOutput("tool_call_results", "");

  RCLCPP_INFO(node_->get_logger(), "AIAction prompt: %s", prompt.c_str());

  ToolCallData& tc_data = ToolCallData::getInstance();
  tools_json_ = tc_data.get_available_tools_json();
  RCLCPP_DEBUG(node_->get_logger(), "tools_json_: %s", tools_json_.c_str());

  ai_session_->user_prompt(prompt, std::string(), enable_reasoning, stream_, tools_json_);
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus AIAction::onRunning() {
  RCLCPP_DEBUG(node_->get_logger(), "%s: OnRunning", name().c_str());

  if (finish_on_next_update_) {
    return BT::NodeStatus::SUCCESS;
  }

  bool abort = false;
  getInput<bool>("abort", abort);
  if (abort) {
    RCLCPP_INFO(node_->get_logger(), "%s: Aborting", name().c_str());
  }

  // If waiting on a tool call to finish
  if (tool_call_wait_) {
    return update_waiting_on_tool_call(abort);

  // Otherwise waiting on the final response for the current prompt
  } else {
    return update_waiting_on_response(abort);
  }
}

void AIAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "AIAction halted");
  if (ai_session_) {
    ai_session_->cancel();
  }
}

BT::NodeStatus AIAction::update_waiting_on_response(bool abort) {
  if (abort) {
    ai_session_->cancel();
    return BT::NodeStatus::SUCCESS;
  }

  // Output the latest sentence(s).
  {
    std::lock_guard<std::mutex> guard(new_sentence_mutex_);
    if (!new_sentence_.empty()) {
      RCLCPP_INFO(node_->get_logger(), "%s: new sentence result: %s", name().c_str(), new_sentence_.c_str());
      setOutput("result", new_sentence_);
      new_sentence_ = std::string();
      return BT::NodeStatus::RUNNING;
    }
  }

  // Check for the completed response
  std::string full_response;
  bool is_tool_call = false;
  AISession::Result result;
  auto finished = ai_session_->is_finished(full_response, is_tool_call, result);

  if (finished) {
    RCLCPP_DEBUG(node_->get_logger(), "AI request finished, result: %s", AISession::result_to_str(result).c_str());

    if (result == AISession::Result::cancelled ||
        result == AISession::Result::timeout ||
        result == AISession::Result::failed ||
        result == AISession::Result::failed_resp_parse_error_general) {
      return BT::NodeStatus::FAILURE;

    } else if (result == AISession::Result::failed_resp_parse_error_tc) {
      ai_session_->report_tool_result("", "", "Failed to parse tool call");
      return BT::NodeStatus::FAILURE;

    } else if (is_tool_call) {
      tool_calls_ = nlohmann::json::parse(full_response);
      tool_call_index_ = 0;

      // Set the full tool call list
      if (parallel_tool_call_support_) {
        setOutput("tool_calls", full_response);
        tool_call_wait_ = true;
        tool_start_time_ = std::chrono::steady_clock::now();
        last_tool_wait_report_ = 0.0f;

      } else {          
        return next_tool_call();
      }          

    } else {
      RCLCPP_INFO(node_->get_logger(), "%s: final result: %s", name().c_str(), streaming_data_buffer_.c_str());

      // Run one more tick to allow the last part of the response to be processed by other BT node(s) (ie TTS)
      setOutput("result", streaming_data_buffer_);
      streaming_data_buffer_.clear();
      finish_on_next_update_ = true;
    }          
  }
  return BT::NodeStatus::RUNNING;
}  

BT::NodeStatus AIAction::update_waiting_on_tool_call(bool abort) {
  std::string tool_call_result;

  if (parallel_tool_call_support_) {
    if (abort) {
      // Report all tool calls as aborted.  This bookends the call for the session context.
      tool_call_result = R"({"result": "Aborted"})";

      for (const auto &tc: tool_calls_) {
        ai_session_->report_tool_result(tc["id"].get<std::string>(), tc["function"]["name"].get<std::string>(),
                                        tool_call_result);
      }

      ai_session_->cancel();

      setOutput("tool_call_name", "");
      setOutput("tool_call_result", "");
      tool_call_wait_ = false;

      return BT::NodeStatus::SUCCESS;
    }

    if (getInput<std::string>("tool_call_results", tool_call_result) &&
        !tool_call_result.empty()) {

      setOutput("tool_call_name", "");
      setOutput("tool_call_result", "");
      setOutput("tool_call_results", "");
      tool_call_wait_ = false;

      nlohmann::json results;
      try {
        results = nlohmann::json::parse(tool_call_result);
        for (const auto &r: results) {
          ai_session_->report_tool_result(r["tool_call_id"], r["tool_call_name"], r["result"]);
        }
      } catch (nlohmann::json::parse_error& ex) {
        RCLCPP_ERROR(node_->get_logger(), "%s, failed parsing tool results, msg: %s, at: %ld", name().c_str(),
                     tool_call_result.c_str(), ex.byte);
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(node_->get_logger(), "Tool calls completed");

      // Get since it could have changed via a tool call
      bool enable_reasoning = false;
      getInput<bool>("enable_reasoning", enable_reasoning);

      // Don't request streaming in case there are subsequent tool calls for the current
      // prompt. It was found that after a few back-to-back calls a new call would not be
      // converted to json format, but rather passed in the native model tool call format.
      // This assumes that at least the first call is correctly output as json.  Re-test
      // after the next vllm update since the tool call parser may change and resolve this.
      ai_session_->tool_calls_finished(enable_reasoning, false, tools_json_);
      return BT::NodeStatus::RUNNING;
    }

  } else {
    if (abort) {
      // Report the call as aborted.  This bookends the tool call for the session context.be
      tool_call_result = R"({"result": "tool call was aborted in progress"})";
      ai_session_->report_tool_result(tool_call_id_, tool_call_name_, tool_call_result);
      ai_session_->cancel();

      setOutput("tool_call_name", "");
      setOutput("tool_call_result", "");
      tool_call_wait_ = false;

      return BT::NodeStatus::SUCCESS;
    }

    if (getInput<std::string>("tool_call_result", tool_call_result) &&
        !tool_call_result.empty()) {

      setOutput("tool_call_name", "");
      setOutput("tool_call_result", "");
      tool_call_wait_ = false;

      ai_session_->report_tool_result(tool_call_id_, tool_call_name_, tool_call_result);
      return next_tool_call();
    }
  }

  auto now = std::chrono::steady_clock::now();
  auto elapsed_sec = std::chrono::duration_cast<std::chrono::duration<float>>(now - tool_start_time_).count();
  if (elapsed_sec > tool_call_timeout_sec_) {
    RCLCPP_ERROR(node_->get_logger(), "Tool call timed-out");
    return BT::NodeStatus::FAILURE;    
  }
  if (elapsed_sec - last_tool_wait_report_ > 2.0f) {
    last_tool_wait_report_ = elapsed_sec;
    RCLCPP_INFO(node_->get_logger(), "Waiting for tool result");
  }        
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus AIAction::next_tool_call() {
  auto num_tools = tool_calls_.size();
  if (tool_call_index_ < num_tools) {
    tool_call_id_ = tool_calls_[tool_call_index_]["id"];
    tool_call_name_ = tool_calls_[tool_call_index_]["function"]["name"];
    nlohmann::json tool_call_args = tool_calls_[tool_call_index_]["function"]["arguments"];

    std::string args = tool_call_args.dump();
    RCLCPP_INFO(node_->get_logger(), "Tool call, index: %ld, id: %s, name: %s, args: %s",
      tool_call_index_, tool_call_id_.c_str(), tool_call_name_.c_str(), args.c_str());

    tool_call_index_++;

    setOutput("result", "");
    setOutput("tool_call_name", tool_call_name_);
    setOutput("tool_call_args", args);
    
    tool_call_wait_ = true;
    tool_start_time_ = std::chrono::steady_clock::now();
    last_tool_wait_report_ = 0.0f;

  } else {
    RCLCPP_INFO(node_->get_logger(), "Tool calls completed");

    // Get since it could have changed via a tool call
    bool enable_reasoning = false;
    getInput<bool>("enable_reasoning", enable_reasoning);

    // Don't request streaming in case there are subsequent tool calls for the current
    // prompt. It was found that after a few back-to-back calls a new call would not be
    // converted to json format, but rather passed in the native model tool call format.
    // This assumes that at least the first call is correctly output as json.  Re-test
    // after the next vllm update since the tool call parser may change and resolve this.
    ai_session_->tool_calls_finished(enable_reasoning, false, tools_json_);
  }
  return BT::NodeStatus::RUNNING;
}

void AIAction::remove_silent_chars(std::string &str) {
  // Remove characters that should be silent
  std::string chars_to_replace = "*";
  for (char c : chars_to_replace) {
    std::replace(str.begin(), str.end(), c, ' ');
  }
}

void AIAction::process_full_response(const std::string& response) {
  streaming_data_buffer_ = response;
  remove_silent_chars(streaming_data_buffer_);
}

void AIAction::add_new_streaming_data(const std::string& delta) {
  streaming_data_buffer_.append(delta);

  if (streaming_data_buffer_.empty()) {
    return;
  }

  remove_silent_chars(streaming_data_buffer_);

  std::string chars_to_replace = "*";
  for (char c : chars_to_replace) {
    std::replace(streaming_data_buffer_.begin(), streaming_data_buffer_.end(),
                  c, ' ');
  }

  auto len = streaming_data_buffer_.length();
  if (len < MIN_SENTENCE_LEN) {
    return;
  }

  // Find the position of the last occurrence of any character from the
  // separators list that is followed by a space.
  std::string separators = ".?!,";
  size_t pos = len;
  do {
    pos = streaming_data_buffer_.find_last_of(separators, pos - 1);
    if (pos == std::string::npos) {
      break;
    }

    auto next_pos = pos + 1;
    if (next_pos < len && (streaming_data_buffer_[next_pos] == ' ' ||
                            streaming_data_buffer_[next_pos] == '\n')) {
      auto sentence = streaming_data_buffer_.substr(0, next_pos);

      RCLCPP_DEBUG(node_->get_logger(), "New Sentence: %s", sentence.c_str());
      {
        std::lock_guard<std::mutex> guard(new_sentence_mutex_);
        new_sentence_.append(sentence);
      }
      streaming_data_buffer_ = streaming_data_buffer_.substr(next_pos);
      break;
    }
  } while (pos > 0);
}
