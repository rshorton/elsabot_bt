#include "ai_session.hpp"

#include <chrono>
#include <regex>
#include <nlohmann/json.hpp>
#include <string>

using namespace std::chrono_literals;

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
  int num_tokens = fetch_token_count(result_json);
  history_.push_back(std::make_unique<SessionMessage_ToolResult>("tool", num_tokens, name, id, result_json));
  current_tokens_ += num_tokens;
}

void AISession::tool_calls_finished() {
#if defined(VLLM_GEMMA)                              
  request_data_ = {{"model", model_},
                   {"stream", true},
                   {"messages", format_prompt()}
                  };

  perform();
#else
  // Not supported                            
  return;                                
#endif
}

void AISession::user_prompt(const std::string &prompt, const std::string &tools_json, const std::string &b64_image) {
  // Fix, account for images
  int num_tokens = fetch_token_count(prompt);
  history_.push_back(std::make_unique<SessionMessage_UserPrompt>("user", num_tokens, prompt, b64_image));
  current_tokens_ += num_tokens;

#if defined(VLLM_GEMMA)                              
  nlohmann::json tools;
  if (tools_json.empty()) {
    tools = nlohmann::json::array();
  } else {
    tools = nlohmann::json::parse(tools_json);
  }    

  request_data_ = {{"model", model_},
                   {"stream", true},
                   {"tools", tools},
                   {"messages", format_prompt()}
                  };
#else
  // We use the /completions endpoint for manual templating
  request_data_ = {{"prompt", format_prompt()},
                   {"stream", true},
                   {"stop", {"</s>", "[INST]"}}
                  };
#endif                            
  perform();
}                                                            

void AISession::perform() {
  is_tool_call_ = false;
  full_response_ = std::string();

  RCLCPP_INFO(logger_, "History context size (approx): %d", current_tokens_);

  auto token_cnt_before = current_tokens_;
  while (current_tokens_ > (max_context_size_ - 1000) && !history_.empty()) {
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

  auto req_text = request_data_.dump();

  // Limit length since b64 images can be large
  RCLCPP_INFO(logger_, "New request: %s", req_text.substr(0, 5000).c_str());

  auto url = host_and_port_ + resource_;

  auto completion_callback = [&](std::string data, CURLcode curl_result) {
    RCLCPP_INFO(logger_, "Completion_callback: length: %ld", data.length());

    AISession::Result result;

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
    promise_.set_value(result);
  };

#if defined(VLLM_GEMMA)
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
          auto j = nlohmann::json::parse(json_str);
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
    while ((pos = data.find(DATA, pos)) != std::string::npos) {
      pos += DATA.length();
      size_t end = data.find("\n\n", pos);
      if (end == std::string::npos) break;

      std::string json_str = data.substr(pos, end - pos);
      if (json_str == "[DONE]") {
        break;
      }

      nlohmann::json j;
      try {
        j = nlohmann::json::parse(json_str);
      } catch (nlohmann::json::parse_error& ex) {
        RCLCPP_ERROR(logger_, "model data parse error args %s, at: %ld", json_str.c_str(), ex.byte);
        return;
      }

      try {
        if (j.contains("choices") && !j["choices"].empty()) {
          if (j["choices"][0]["delta"].contains("content")) {
            const std::string &delta = j["choices"][0]["delta"]["content"];
            full_response_ += delta;
            if (callback_data_) {
              callback_data_(delta);
            }

          } else if (j["choices"][0]["delta"].contains("tool_calls")) {
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
          } else if (j["choices"][0]["finish_reason"] == "tool_calls") {
            // End of tool calls.  Assemble them into one object
            auto tool_calls = nlohmann::json::parse(R"( {"choices": [ {"delta": {"tool_calls": []} } ]} )");
            auto full_response_obj = tool_calls;

            RCLCPP_DEBUG(logger_, "tc parse, prefix: %s", tool_calls.dump().c_str());

            for (auto const& [key, value] : toolcall_) {
              RCLCPP_DEBUG(logger_, "tc parse, key: %ld, value: %s, args: %s",
                          key, value.dump().c_str(), toolcall_args_[key].c_str());

              nlohmann::json tc_temp = value;
              tc_temp["function"]["arguments"] = nlohmann::json::parse(toolcall_args_[key]);
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
            RCLCPP_DEBUG(logger_, "tc parse, tool_call_: %s", tool_call_.c_str());
          }
        }
      } catch (...) {
        RCLCPP_ERROR(logger_, "Failed parsing response data fragment");
      }
      pos = end;
    }
  };
#else
  auto data_callback = [&](std::string data) {
    // std::cout << "data_callback: " << data << std::endl;
    //  Example:  data:
    //    delta update
    //  data: {"index":0,"content":","tokens":[29871],"stop":false,"id_slot":-1,"tokens_predicted":44,"tokens_evaluated":239}\n
    //
    //  When finished:  "stop": true

    size_t pos = 0;
    while ((pos = data.find("data: ", pos)) != std::string::npos) {
      pos += 6;
      size_t end = data.find("\n", pos);
      if (end == std::string::npos) {
        break;
      }
      try {
        std::string c = nlohmann::json::parse(data.substr(pos, end - pos))["content"];
        //std::cout << c << std::flush;
        full_response_ += c;
        callback_data_(c);

      } catch (...) {
      }
      pos = end;
    }
  };
#endif

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
    RCLCPP_INFO(logger_, "Future is ready, result: %s", result_to_str(result).c_str());

    if (result == AISession::Result::success) {
      is_tool_call = is_tool_call_;
      if (is_tool_call_) {
        response = tool_call_;  
      } else {
        response = full_response_;
      }

      int num_tokens = fetch_token_count(response);
      if (is_tool_call) {
        history_.push_back(std::make_unique<SessionMessage_ToolRequest>("assistant", num_tokens, full_response_));
      } else {
        history_.push_back(std::make_unique<SessionMessage_Response>("assistant", num_tokens, full_response_));
      }        
      current_tokens_ += num_tokens;
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
    default:
      return "unknown";
  }
}

int AISession::fetch_token_count(const std::string& text) const {
#if defined(VLLM_GEMMA)
  int count = (int)text.length() / 4;

#else
  nlohmann::json body = {{"content", text}};
  // Default count to length/4 if cannot request
  int count = (int)text.length() / 4;

  auto completion_callback = [&](std::string data, CURLcode result) {
    RCLCPP_DEBUG(logger_, "Completion_callback (token count): %s", data.c_str());

    if (result == CURLE_OK) {
      RCLCPP_DEBUG(logger_, "Token count request successful. Data length: %ld bytes", data.length());
      count = (int)nlohmann::json::parse(data)["tokens"].size();
    } else if (result == CURLE_OPERATION_TIMEDOUT) {
      RCLCPP_ERROR(logger_, "Token count request timed-out.");
    } else {
      RCLCPP_ERROR(logger_, "Token count request failed: %s", curl_easy_strerror(result));
    }
  };

  auto url = host_address_and_port_ + "/tokenize";
  auto timeout = 8000;

  request_ = std::make_unique<HttpRequest>(
      false, url, timeout, std::string(), body.dump(), false,
      completion_callback, nullptr);
  request_->perform();
#endif    
  return count;
}

#if defined(VLLM_GEMMA)
nlohmann::json AISession::format_prompt() const {

  nlohmann::json prompt = nlohmann::json::array();
  prompt.push_back({{"role", "system"}, {"content", system_prompt_}});

  auto last_index = (long int)history_.size() - 1;

  for (auto it = history_.begin(); it != history_.end(); ++it) {
    auto index = std::distance(history_.begin(), it);
    auto m = (*it)->get_json(last_index == index);
    prompt.push_back(m);
  }
  return prompt;
}

#else
// Manual Llama 2 Template Builder
std::string AISession::format_prompt() {
  std::string prompt =
      "<s>[INST] <<SYS>>\n" + system_prompt_ + "\n<</SYS>>\n\n";
  
  bool first_user = true;

  for (const auto& msg : history_) {
    if (msg.role == "user") {
      if (!first_user) {
        prompt += "<s>[INST] ";
      }

      prompt += msg.content + " [/INST] ";
      first_user = false;
    } else {
      prompt += msg.content + " </s>";
    }
  }
  return prompt;
}
#endif


