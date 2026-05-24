#include "ai_session.hpp"

#include <chrono>
#include <regex>
#include <nlohmann/json.hpp>
#include <string>

using namespace std::chrono_literals;
using json = nlohmann::json;

AISession::AISession(const std::string &model, int max_context_size, const std::string &auth_token,
                     const std::string &host_and_port, const std::string &resource, 
                     long timeout_ms, const std::string &system_prompt, rclcpp::Logger logger,
                     CallbackData callback_data):
  model_(model),
  max_context_size_(max_context_size),
  auth_token_(auth_token),
  host_and_port_(host_and_port),
  resource_(resource),
  timeout_ms_(timeout_ms),
  system_prompt_(system_prompt),
  logger_(logger),
  callback_data_(std::move(callback_data)) {
}

AISession::~AISession() {
}

void AISession::report_tool_result(const std::string &id, const std::string &name, const std::string &result_json) {
  // Limit length since b64 images can be large
  RCLCPP_INFO(logger_, "Tool call result: %s", result_json.substr(0, 5000).c_str());
  auto has_image = result_json.find("image_url") != std::string::npos;
  history_.push_back(std::make_unique<SessionMessage_ToolResult>("tool", 0, name, id, result_json, has_image));
}

void AISession::tool_calls_finished(bool stream, const std::string &tools_json) {
  json tools;
  if (tools_json.empty()) {
    tools = json::array();
  } else {
    tools = json::parse(tools_json);
  }

  request_data_ = {{"model", model_},
                   {"tools", tools},
                   {"tool_choice", "auto"},
                   {"messages", format_prompt()},
                   {"stream", stream},
                   {"extra_body", {{"chat_template_kwargs", {{"enable_thinking", true}}}}}
                  };
  // Need to request usage streaming used
  if (stream) {
    request_data_["stream_options"] = {{"include_usage", true}};
  }                  

  perform();
}

void AISession::user_prompt(const std::string &prompt, bool stream, const std::string &tools_json, const std::string &b64_image) {
  history_.push_back(std::make_unique<SessionMessage_UserPrompt>("user", 0, prompt, b64_image));

  json tools;
  if (tools_json.empty()) {
    tools = json::array();
  } else {
    tools = json::parse(tools_json);
  }    

  request_data_ = {{"model", model_},
                   {"tools", tools},
                   {"tool_choice", "auto"},
                   {"messages", format_prompt()},
                   {"stream", stream},
                   {"extra_body", {{"chat_template_kwargs", {{"enable_thinking", true}}}}}
                  };
  // Need to request usage streaming used
  if (stream) {
    request_data_["stream_options"] = {{"include_usage", true}};
  }                  
  perform();
}                                                            

void AISession::perform() {
  is_tool_call_ = false;
  full_response_ = std::string();

  response_parse_error_ = false;
  response_parse_error_tool_call_ = false;
  token_usage_cur_msg_ = TokenUsage();

  prune_message_history_as_needed();

  auto req_text = request_data_.dump();

  // Limit length since b64 images can be large
  RCLCPP_INFO(logger_, "New request: %s", req_text.substr(0, 5000).c_str());

  auto url = host_and_port_ + resource_;

  auto completion_callback = [&](std::string data, CURLcode curl_result) {
    RCLCPP_DEBUG(logger_, "Completion_callback: length: %ld", data.length());

    if (complete_.load()) {
      RCLCPP_DEBUG(logger_, "Error, completion_callback after complete");
      return;
    }

    AISession::Result result;

    if (response_parse_error_tool_call_) {
      result = Result::failed_resp_parse_error_tc;
      RCLCPP_INFO(logger_, "Request failed, tool call parse error");
    } else if (response_parse_error_) {
      result = Result::failed_resp_parse_error_general;
      RCLCPP_INFO(logger_, "Request failed, general parse error");
    } else {
      if (curl_result == CURLE_OK) {
        RCLCPP_INFO(logger_, "Request successful");
        result = Result::success;
      } else if (curl_result == CURLE_ABORTED_BY_CALLBACK) {
        RCLCPP_INFO(logger_, "Model request was cancelled");
        result = Result::cancelled;
      } else if (curl_result == CURLE_OPERATION_TIMEDOUT) {
        RCLCPP_ERROR(logger_, "Model request timed-out");
        result = Result::timeout;
      } else {
        RCLCPP_ERROR(logger_, "Model request failed: %s", curl_easy_strerror(curl_result));
        result = Result::failed;
      }
    }
    complete_.store(true);
    promise_.set_value(result);
  };

  auto data_callback = [&](std::string data) {

    // vLLM sends data in Server-Sent Events (SSE) format: "data: {...}"
    std::cout << "data_callback: " << data << std::endl;

    size_t pos = 0;

    const std::string ERROR = "error: ";
    pos = data.find(ERROR, pos);
    if (pos != std::string::npos) {
      std::string error_msg;
      pos += ERROR.length();
      size_t end = data.find("\n\n", pos);
      if (end != std::string::npos) {
        std::string json_str = data.substr(pos, end - pos);
        try {
          auto j = json::parse(json_str);
          if (j.contains("message") && !j["message"].empty()) {
            error_msg = j["message"];
        }
      } catch (...) {} 
      }
      RCLCPP_ERROR(logger_, "Model request error: %s", error_msg.c_str());
      return;
    }

    pos = 0;
    const std::string DATA = "data: ";
    if (data.find(DATA, pos) != std::string::npos) {
      pos = 0;
      while ((pos = data.find(DATA, pos)) != std::string::npos) {
        pos += DATA.length();
        size_t end = data.find("\n\n", pos);
        if (end == std::string::npos) {
          break;
        }

        std::string json_str = data.substr(pos, end - pos);
        if (json_str == "[DONE]") {
          break;
        }

        process_message(json_str, token_usage_cur_msg_);
        pos = end;
      }
    } else {
      process_message(data, token_usage_cur_msg_);
    }      
  };

  complete_.store(false);
  promise_ = std::move(std::promise<AISession::Result>());
  future_ = std::move(promise_.get_future());

  request_ = std::make_unique<HttpRequest>(true, url, timeout_ms_, std::string(), req_text,
                                            true, completion_callback, data_callback);
  request_->perform();
}

bool AISession::is_finished(std::string &response, bool &is_tool_call, AISession::Result &result) {
  if (!request_) {
    result = AISession::Result::failed;
    return true;
  }

  auto status = future_.wait_for(0ms);

  if (status == std::future_status::ready) {
    result = future_.get();
    RCLCPP_INFO(logger_, "Future is ready, result: %s, full_response: %s",
                result_to_str(result).c_str(), full_response_.c_str());

    if (result == AISession::Result::success) {
      is_tool_call = is_tool_call_;
      if (is_tool_call_) {
        response = tool_call_;  
      } else {
        response = full_response_;
      }

      // Set the token size for the last prompt/tool response based on the prompt_token delta since the previous msg
      update_tokens_for_last_prompt(token_usage_cur_msg_.prompt_tokens_ - token_usage_.prompt_tokens_);
      current_tokens_ = token_usage_cur_msg_.total_tokens_;
      token_usage_ = token_usage_cur_msg_;

      if (is_tool_call) {
        history_.push_back(std::make_unique<SessionMessage_ToolRequest>("assistant", token_usage_cur_msg_.completion_tokens_, full_response_));
      } else {
        history_.push_back(std::make_unique<SessionMessage_Response>("assistant", token_usage_cur_msg_.completion_tokens_, full_response_));
      }        
    }
    return true;
  }
  return false;
}

void AISession::cancel() {
  if (request_) {
    request_->cancel();
  }
}

std::string AISession::result_to_str(AISession::Result result) {
  switch (result) {
    case AISession::Result::success:
      return "success";
    case AISession::Result::cancelled:
      return "cancelled";
    case AISession::Result::timeout:
      return "timeout";
    case AISession::Result::failed:
      return "failed";
    case AISession::Result::failed_resp_parse_error_general:
      return "failed, general message parse error";
    case AISession::Result::failed_resp_parse_error_tc:
      return "failed, tool call parse error";
    default:
      return "unknown";
  }
}

// Rough estimate - doesn't handle images
int AISession::fetch_token_count(const std::string& text) const {
  return (int)text.length() / 4;
}

void AISession::update_tokens_for_last_prompt(size_t tokens) {
  if (history_.size()) {
    history_.back()->set_token_count(tokens);
  }
}

json AISession::format_prompt() const {

  json prompt = json::array();
  prompt.push_back({{"role", "system"}, {"content", system_prompt_}});

  auto last_index = (long int)history_.size() - 1;

  RCLCPP_INFO(logger_, "Context summary:");
  for (auto it = history_.begin(); it != history_.end(); ++it) {
    auto index = std::distance(history_.begin(), it);
    auto m = (*it)->get_json(last_index == index);
    prompt.push_back(m);
    RCLCPP_INFO(logger_, "%s", (*it)->get_description().c_str());
  }
  return prompt;
}

void AISession::process_message(const std::string& msg_json, TokenUsage &usage) {      
  json j;
  try {
    j = json::parse(msg_json);
  } catch (json::parse_error& ex) {
    RCLCPP_ERROR(logger_, "model message parse error, msg: %s, at: %ld", msg_json.c_str(), ex.byte);
    return;
  }

  try {
    if (j.contains("usage")) {
      if (j["usage"].contains("prompt_tokens")) {
        usage.prompt_tokens_ = j["usage"]["prompt_tokens"];
      }
      if (j["usage"].contains("total_tokens")) {
        usage.total_tokens_ = j["usage"]["total_tokens"];
      }
      if (j["usage"].contains("completion_tokens")) {
        usage.completion_tokens_ = j["usage"]["completion_tokens"];
      }
    }

    if (!j.contains("choices") || j["choices"].empty()) {
      RCLCPP_DEBUG(logger_, "message missing choices, msg: %s", msg_json.c_str());
      return;
    }

    // Streaming case where deltas are received
    if (j["choices"][0].contains("delta")) {
      if (j["choices"][0]["delta"].contains("content")) {
        const std::string &delta = j["choices"][0]["delta"]["content"];
        full_response_ += delta;
        if (callback_data_) {
          callback_data_(delta);
        }

      } else if (j["choices"][0]["delta"].contains("tool_calls")) {
        try {
          const auto &tc = j["choices"][0]["delta"]["tool_calls"][0];
          size_t index = tc["index"];
          RCLCPP_DEBUG(logger_, "tc parse, index: %ld, base: %s", index, tc.dump().c_str());

          if (tc.contains("id")) {
            toolcall_[index] = tc;
            toolcall_args_[index] = "";
          } else {
            // Aggregate the argument fragments
            std::string arg_frag = tc["function"]["arguments"];
            toolcall_args_[index] += arg_frag;
            RCLCPP_DEBUG(logger_, "tc parse, args, index: %ld, frag: %s, args accum: %s", index,
                        arg_frag.c_str(), toolcall_args_[index].c_str());
          }
        } catch (...) {
          RCLCPP_ERROR(logger_, "Failed parsing model tool call message (delta)");
          response_parse_error_ = true;
          response_parse_error_tool_call_ = true;
        }          

      } else if (j["choices"][0]["finish_reason"] == "tool_calls") {
        try {
          // End of tool calls.  Assemble them into one object
          auto tool_calls = json::parse(R"( {"choices": [ {"delta": {"tool_calls": []} } ]} )");
          auto full_response_obj = tool_calls;

          RCLCPP_DEBUG(logger_, "tc parse, prefix: %s", tool_calls.dump().c_str());

          for (auto const& [key, value] : toolcall_) {
            RCLCPP_DEBUG(logger_, "tc parse, key: %ld, value: %s, args: %s",
                        key, value.dump().c_str(), toolcall_args_[key].c_str());

            json tc_temp = value;
            tc_temp["function"]["arguments"] = json::parse(toolcall_args_[key]);
            tool_calls["choices"][0]["delta"]["tool_calls"].push_back(tc_temp);

            // Insert the args as json for the full_response since it that format is required
            // for the message history
            tc_temp = value;
            tc_temp["function"]["arguments"] = toolcall_args_[key];
            full_response_obj["choices"][0]["delta"]["tool_calls"].push_back(tc_temp);
          }

          tool_call_ = tool_calls["choices"][0]["delta"]["tool_calls"].dump();
          is_tool_call_ = true;

          full_response_ = full_response_obj["choices"][0]["delta"]["tool_calls"].dump();
          RCLCPP_DEBUG(logger_, "tc parse, tool_call_ (from deltas): %s", tool_call_.c_str());
        } catch (...) {
          RCLCPP_ERROR(logger_, "Failed parsing model tool call message");
          response_parse_error_ = true;
          response_parse_error_tool_call_ = true;
        }          

      } else if (j["choices"][0]["finish_reason"] == "stop") {        
        RCLCPP_DEBUG(logger_, "chat message end (from deltas): %s", full_response_.c_str());
      }              

    // Non-streaming cases
    } else if (j["choices"][0]["finish_reason"] == "tool_calls") {

      if (j["choices"][0].contains("message") && 
          j["choices"][0]["message"].contains("tool_calls")) {

        RCLCPP_DEBUG(logger_, "finish_reason tool_calls with a message");

        // Full response needs to be as was received with args as json
        full_response_ = j["choices"][0]["message"]["tool_calls"].dump();

        // Parse the args of each tool call
        auto tool_calls = json::array();

        auto tc_array = j["choices"][0]["message"]["tool_calls"];
        for (const auto &tc: tc_array) {
          std::string args = tc["function"]["arguments"];
          tool_calls.push_back(tc);

          json arg_obj;
          try {
            arg_obj = json::parse(args);
          } catch (json::parse_error& ex) {
            RCLCPP_ERROR(logger_, "failed parsing tool call args: %s, at: %ld", args.c_str(), ex.byte);
            return;
          }
          tool_calls.back()["function"]["arguments"] = arg_obj;
        }

        tool_call_ = tool_calls.dump();
        is_tool_call_ = true;
        RCLCPP_DEBUG(logger_, "tc parse, tool_call_(from non-delta): %s", tool_call_.c_str());
      }              

    } else if (j["choices"][0]["finish_reason"] == "stop") {
      if (j["choices"][0].contains("message") && 
          j["choices"][0]["message"].contains("content")) {
          full_response_ =  j["choices"][0]["message"]["content"];
          RCLCPP_INFO(logger_, "chat content (from non-delta): %s", full_response_.c_str());
          if (callback_data_) {
            callback_data_(full_response_);
          }          
      }
    }
  } catch (...) {
    RCLCPP_ERROR(logger_, "Failed parsing model response message");
    response_parse_error_ = true;
  }
}

std::string AISession::session_message_to_str(AISession::SessionMessageType msg_type) {
  switch (msg_type) {
    case AISession::SessionMessageType::prompt:
      return "prompt";
    case AISession::SessionMessageType::response:
      return "response";
    case AISession::SessionMessageType::tool_request:
      return "tool_request";
    case AISession::SessionMessageType::tool_result:
      return "tool_response";
    default:
      return "unknown";
  }
}

void AISession::prune_message_history_as_needed() {
  RCLCPP_INFO(logger_, "Context size (est): %d, | Usage from last response, prompt_tokens: %ld, completion_tokens: %ld, total_tokens: %ld",
    current_tokens_, token_usage_.prompt_tokens_, token_usage_.completion_tokens_, token_usage_.total_tokens_);

  auto token_cnt_before = current_tokens_;

  if (current_tokens_ > max_context_size_ && !history_.empty()) {
    // Remove images from oldest to newest
    for (auto &item: history_) {
      if (item->has_image()) {
        current_tokens_ -= item->get_token_count();
        item->set_use_image(false);
      }
    }      
  }

  // If history still too large, drop message pairs
  while (current_tokens_ > max_context_size_ && !history_.empty()) {
    current_tokens_ -= history_.front()->token_count_;
    history_.pop_front();
    // Pop by prompt/response pairs
    if (!history_.empty()) {
      current_tokens_ -= history_.front()->token_count_;
      history_.pop_front();
    }
  }

  if (token_cnt_before != current_tokens_) {
    RCLCPP_DEBUG(logger_, "Purged history to reduce context size: before: %d, after: %d",
      token_cnt_before, current_tokens_);
  }
}


void AISession::SessionMessage::get_description(std::stringstream &ss) const {
  ss << "type: " << session_message_to_str(msg_type_) << ", role: " << role_ << ", tokens: " << token_count_;
}


std::string AISession::SessionMessage_UserPrompt::get_description() const {
  std::stringstream ss;
  SessionMessage::get_description(ss);
  ss << ", prompt: " << prompt_ << ", has_image: " << (b64_image_.empty()? "N": "Y");
  return ss.str();
}

std::string AISession::SessionMessage_Response::get_description() const {
  std::stringstream ss;
  SessionMessage::get_description(ss);
  ss << ", response: " << response_;
  return ss.str();
}


std::string AISession::SessionMessage_ToolRequest::get_description() const {
  std::stringstream ss;
  SessionMessage::get_description(ss);
  ss <<  ", tool_call_json: " << tool_call_json_;
  return ss.str();
}


std::string AISession::SessionMessage_ToolResult::get_description() const {
  std::stringstream ss;
  SessionMessage::get_description(ss);

  ss << ", tool: " << name_  << ", using_image: " << (has_image() && use_image());
  if (!tool_result_json_.empty()) {
    auto j = nlohmann::json::parse(tool_result_json_);
    remove_images(j, " [omitted image]");
    ss << ", tool_result_json: " << j.dump();
  }
  return ss.str();
}
