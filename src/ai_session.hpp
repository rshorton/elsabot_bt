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

  void user_prompt(const std::string &prompt, bool stream, const std::string &tools_json, const std::string &b64_image = std::string());
  
  void report_tool_result(const std::string &id, const std::string &name, const std::string &result_json);
  void tool_calls_finished(bool stream, const std::string &tools_json);

  void cancel();

  bool is_finished(std::string &response, bool &is_tool_call, AISession::Result &result);

  static std::string result_to_str(AISession::Result result);

 private:
  void perform();
  void process_message(const std::string& msg_json);
  int fetch_token_count(const std::string& text) const;

#if defined(VLLM_GEMMA)
  nlohmann::json format_prompt() const;
#else
  std::string format_prompt();
#endif

  class SessionMessage {
  public:
    SessionMessage(const std::string role, const int token_count):
      role_(role),
      token_count_(token_count)
    {}

    virtual nlohmann::json get_json(bool) {
        return {{"role", role_}};
    }

    std::string role_;
    int token_count_;
  };

  class SessionMessage_UserPrompt: public SessionMessage {
  public:
    SessionMessage_UserPrompt(const std::string role, const int token_count, const std::string &prompt,
                              const std::string &b64_image):
      SessionMessage(role, token_count),
      prompt_(prompt),
      b64_image_(b64_image)
    {}

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      if (b64_image_.empty()) {
        m["content"] = prompt_;
      } else {
        nlohmann::json msg = nlohmann::json::array();
        msg.push_back({{"type", "image_url"}, {"image_url", {{"url", b64_image_}}}});
        msg.push_back({{"type", "text"}, {"text", prompt_}});
        m["content"] = msg;
      }
      return m;
    }

    std::string prompt_;
    std::string b64_image_;
  };

  class SessionMessage_Response: public SessionMessage {
  public:
    SessionMessage_Response(const std::string role, const int token_count, const std::string &response):
      SessionMessage(role, token_count),
      response_(response)
    {}

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
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

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      m["tool_calls"] = nlohmann::json::parse(tool_call_json_);
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

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      m["tool_call_id"] = id_;
      m["name"] = name_;
      // Only send tool result once (ie. for the turn it was created)
      if (!reported_) {
        if (name_ == "get_camera_frame") {
          m["content"] = nlohmann::json::parse(tool_result_json_);
        } else {
          m["content"] = tool_result_json_;
        }          
        //reported_ = true;
      } else {
        m["content"] = R"({})";
      }        
      return m;
    }

    std::string name_;
    std::string id_;
    std::string tool_result_json_;
    bool reported_{false};
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

  std::map<size_t, nlohmann::json> toolcall_;
  std::map<size_t, std::string> toolcall_args_;
};

#endif  // AI_SESSION_H
