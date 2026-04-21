#ifndef AI_SESSION_H
#define AI_SESSION_H

#include "rclcpp/rclcpp.hpp"

#include "http_request.hpp"
#include <nlohmann/json.hpp>

#include <iostream>
#include <string>
#include <deque>
#include <functional>
#include <future>
#include <memory>

#define VLLM_GEMMA

class AISession {
 public:
  using CallbackData = std::function<void(const std::string &data)>;

  enum class Result { success, cancelled, timeout, failed };

  AISession(const std::string &model, int max_context_size, const std::string &auth_token,
         const std::string &host_and_port, const std::string &resource, long timeout_ms,
         const std::string &system_prompt, rclcpp::Logger logger,
         CallbackData callback_data);
  ~AISession();

  void user_prompt(const std::string &prompt, const std::string &tools_json);
  void send_tool_result(const std::string &id, const std::string &name, const std::string &result_json);

  void cancel();

  bool is_finished(std::string &response, bool &is_tool_call, AISession::Result &result);

  static std::string result_to_str(AISession::Result result);

 private:
  void perform();
  int fetch_token_count(const std::string& text);

#if defined(VLLM_GEMMA)
  nlohmann::json format_prompt();
#else
  std::string format_prompt();
#endif

  class SessionMessage {
  public:
    SessionMessage(const std::string role, const int token_count):
      role_(role),
      token_count_(token_count)
    {}

    virtual nlohmann::json get_json() {
        return {{"role", role_}};
    }

    std::string role_;
    int token_count_;
  };

  class SessionMessage_UserPrompt: public SessionMessage {
  public:
    SessionMessage_UserPrompt(const std::string role, const int token_count, const std::string &prompt):
      SessionMessage(role, token_count),
      prompt_(prompt)
    {}

    nlohmann::json get_json() override {
      nlohmann::json m = SessionMessage::get_json();
      m["content"] = prompt_;
      return m;
    }

    std::string prompt_;
  };

  class SessionMessage_Response: public SessionMessage {
  public:
    SessionMessage_Response(const std::string role, const int token_count, const std::string &response):
      SessionMessage(role, token_count),
      response_(response)
    {}

    nlohmann::json get_json() override {
      nlohmann::json m = SessionMessage::get_json();
      m["content"] = response_;
      return m;
    }

    std::string response_;
  };

  class SessionMessage_ToolRequest: public SessionMessage {
  public:
    SessionMessage_ToolRequest(const std::string role, const int token_count, const std::string &tool_call_json):
      SessionMessage(role, token_count),
      tool_call_json_(tool_call_json)
    {}

    nlohmann::json get_json() override {
      nlohmann::json m = SessionMessage::get_json();
      m["tool_calls"] = nlohmann::json::array();
      m["tool_calls"].push_back(nlohmann::json::parse(tool_call_json_));
      return m;
    }

    std::string tool_call_json_;
  };

  class SessionMessage_ToolResult: public SessionMessage {
  public:
    SessionMessage_ToolResult(const std::string role, const int token_count, const std::string &name,
      const std::string &id, const std::string &tool_result_json):
      SessionMessage(role, token_count),
      name_(name),
      id_(id),
      tool_result_json_(tool_result_json)
    {}

    nlohmann::json get_json() override {
      nlohmann::json m = SessionMessage::get_json();
      m["tool_call_id"] = id_;
      m["name"] = name_;
      m["content"] = tool_result_json_;
      return m;
    }

    std::string name_;
    std::string id_;
    std::string tool_result_json_;
  };

  const std::string model_;
  int max_context_size_;
  std::string auth_token_;
  const std::string host_and_port_;
  const std::string resource_;
  long timeout_ms_;
  const std::string system_prompt_;

  rclcpp::Logger logger_;

  CallbackData callback_data_;
  
  std::deque<std::unique_ptr<SessionMessage>> history_;
  int current_tokens_{0};

  nlohmann::json request_data_;
  AISession::Result result_{AISession::Result::failed};
  std::string full_response_;
  std::string tool_call_;
  bool is_tool_call_{false};

  std::promise<AISession::Result> promise_;
  std::future<AISession::Result> future_;

  std::unique_ptr<HttpRequest> request_;

  nlohmann::json tool_call_build_;
  std::string tool_call_args_json_;

};

#endif  // AI_SESSION_H
