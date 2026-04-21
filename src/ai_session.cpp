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

void AISession::send_tool_result(const std::string &id, const std::string &name, const std::string &result_json) {
  int num_tokens = fetch_token_count(result_json);
  history_.push_back(std::make_unique<SessionMessage_ToolResult>("tool", num_tokens, name, id, result_json));
  current_tokens_ += num_tokens;

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

void AISession::user_prompt(const std::string &prompt, const std::string &tools_json) {
  int num_tokens = fetch_token_count(prompt);
  history_.push_back(std::make_unique<SessionMessage_UserPrompt>("user", num_tokens, prompt));
  current_tokens_ += num_tokens;

#if defined(VLLM_GEMMA)                              
  nlohmann::json tools = nlohmann::json::parse(tools_json);

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

  RCLCPP_INFO(logger_, "New request: %s", req_text.c_str());

  auto url = host_and_port_ + resource_;

  auto completion_callback = [&](std::string data, CURLcode curl_result) {
    RCLCPP_INFO(logger_, "Completion_callback: length: %ld", data.length());

    AISession::Result result;

    if (curl_result == CURLE_OK) {
      RCLCPP_INFO(logger_, "Request successful");
      result = Result::success;
    } else if (curl_result == CURLE_ABORTED_BY_CALLBACK) {
      RCLCPP_DEBUG(logger_, "Model request was cancelled");
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
        tool_call_args_json_ = std::string("");
        break;
      }

      try {
        if (json_str.find("tool_calls\":[") != std::string::npos) {

          const std::string TC_SEARCH_STR = "delta\":{\"tool_calls\":[";
          if ((pos = json_str.find(TC_SEARCH_STR)) != std::string::npos) {
            // When streaming output from the model is enabled, the tool_calls are also streamed.
            // However, only the 'argument' property is broken into fragments.  See
            // https://github.com/vllm-project/vllm/issues/9693
            // The first part includes the toolcall id and everything but the arguments.  The arguments
            // are specified as an empty string. The first part is valid json.  Subsequent fragments
            // include some of the overall toolcall previously sent, but with the 'arguments'
            // property incrementally sent.  The fragments are invalid json.
            // 
            // To parse:
            // 1. For the fragment including the 'id', consider that the base of the json.
            // 2. For each part following, use a regex to extract the contents of the 'arguments'
            //    property and append to an accumulated value.
            // 3. Check for "finish_reason": "tool_calls" to signal the end of the fragments, and
            //    then insert the accumulated args into the base json.

            if (json_str.find("\"id\"", pos) != std::string::npos) {
              
              tool_call_build_ = nlohmann::json::parse(json_str);
              RCLCPP_DEBUG(logger_, "tc parse base, %s", tool_call_build_.dump().c_str());
            } else {
              RCLCPP_DEBUG(logger_, "tc parse frag");

              std::regex pattern(".*arguments\":\"(.*)\"}}].*");
              std::smatch match;
              if (std::regex_search(json_str, match, pattern) && match.size() > 1) {
                tool_call_args_json_ += match[1].str();
                RCLCPP_DEBUG(logger_, "tc parse, arg fragment: %s, args accum: %s", match[1].str().c_str(), tool_call_args_json_.c_str());
              }
            }
          }

        } else {
          auto j = nlohmann::json::parse(json_str);
          if (j.contains("choices") && !j["choices"].empty()) {
            if (j["choices"][0]["finish_reason"] == "tool_calls") {
              // All of the fragments for the tool_call should have been received.
              RCLCPP_DEBUG(logger_, "tc parse, accumulated args: %s", tool_call_args_json_.c_str());
              try {
                // The assembed args have /" instead of just " around the strings.  Replace with just "
                tool_call_args_json_ = std::regex_replace(tool_call_args_json_, std::regex(R"(\\\")"), R"(")");
              } catch (const std::regex_error& e) {
                RCLCPP_ERROR(logger_, "tc parse error, ex: %s", e.what());  
              }                
              RCLCPP_DEBUG(logger_, "tc parse, accumulated args (escapes removed): %s", tool_call_args_json_.c_str());
             
              nlohmann::json args;
              try {
                args = nlohmann::json::parse(tool_call_args_json_);
              } catch (nlohmann::json::parse_error& ex) {
                RCLCPP_ERROR(logger_, "tc parse error args %s, at: %ld", tool_call_args_json_.c_str(), ex.byte);
                continue;
              }

              // Build a full tool call with the string format of the args since this needs to go into the message history
              auto tool_call_with_str_args = tool_call_build_;
              tool_call_with_str_args["choices"][0]["delta"]["tool_calls"][0]["function"]["arguments"] = tool_call_args_json_;
              full_response_ = tool_call_with_str_args["choices"][0]["delta"]["tool_calls"][0].dump();

              // Build a full tool call with the args as an object
              tool_call_build_["choices"][0]["delta"]["tool_calls"][0]["function"]["arguments"] = args;
              RCLCPP_DEBUG(logger_, "tc parse, with inserted args: %s", tool_call_build_.dump().c_str());

              // Now pluck-out the toolcall details
              auto tool_call = tool_call_build_["choices"][0]["delta"]["tool_calls"][0];
              tool_call_ = tool_call.dump();
              is_tool_call_ = true;

              RCLCPP_DEBUG(logger_, "Toolcall: %s", tool_call_.c_str());

            } else if (j["choices"][0]["delta"].contains("content")) {
              std::string delta = j["choices"][0]["delta"]["content"];
              full_response_ += delta;
              callback_data_(delta);
            }            
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
      response = tool_call_;
      is_tool_call = is_tool_call_;

      int num_tokens = fetch_token_count(full_response_);
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

int AISession::fetch_token_count(const std::string& text) {
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
nlohmann::json AISession::format_prompt() {

  nlohmann::json prompt = nlohmann::json::array();
  prompt.push_back({{"role", "system"}, {"content", system_prompt_}});

  for (const auto& msg : history_) {
    auto m = msg->get_json();
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

