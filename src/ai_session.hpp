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

class AISession {
 public:
  using CallbackData = std::function<void(const std::string &data)>;

  enum class Result { success, cancelled, timeout, failed, failed_resp_parse_error_general, failed_resp_parse_error_tc };
  enum class SessionMessageType { prompt, response, tool_request, tool_result};
 
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

  static std::string session_message_to_str(AISession::SessionMessageType);

 private:
  class TokenUsage {
  public:    
    TokenUsage() {};

    TokenUsage(size_t prompt_tokens, size_t completion_tokens, size_t total_tokens):
      prompt_tokens_(prompt_tokens),
      completion_tokens_(completion_tokens),
      total_tokens_(total_tokens)
    {}

    size_t prompt_tokens_{0};       // The number of tokens sent to the model
    size_t completion_tokens_{0};   // The number of tokens the model generated in last response
    size_t total_tokens_{0};        // Sum of prompt + completion tokens
  };

  void perform();
  void process_message(const std::string& msg_json, TokenUsage &usage);
  int fetch_token_count(const std::string& text) const;
  void update_tokens_for_last_prompt(size_t tokens);
  void prune_message_history_as_needed();

  nlohmann::json format_prompt() const;
  
  class SessionMessage {
  public:
    SessionMessage(SessionMessageType msg_type, const std::string &role, const int token_count, bool has_image):
      msg_type_(msg_type),
      role_(role),
      token_count_(token_count),
      has_image_(has_image)
    {}

    virtual nlohmann::json get_json(bool) {
        return {{"role", role_}};
    }
    void set_token_count(size_t count) {
      token_count_ = count;
    }

    virtual std::string get_description() {
      std::stringstream ss;
      ss << "type: " << session_message_to_str(msg_type_) << ", role: " << role_ << ", tokens: " << token_count_;
      return ss.str();
    }

    SessionMessageType get_type() const {
      return msg_type_;
    }

    bool has_image() const {
      return has_image_ && use_image_;
    }

    bool use_image() const {
      return use_image_;
    }

    void set_use_image(bool use) {
      use_image_ = use;
    }
    
    size_t get_token_count() const {
      // If the image is omitted, the token count
      // will be > 0 but small.  
      if (has_image_ && !use_image_) {
        return 0;
      } else {
        return token_count_;
      }
    }

    SessionMessageType msg_type_;
    std::string role_;
    int token_count_;
    bool has_image_;
    bool use_image_{true};
  };

  class SessionMessage_UserPrompt: public SessionMessage {
  public:
    SessionMessage_UserPrompt(const std::string role, const int token_count, const std::string &prompt,
                              const std::string &b64_image):
      SessionMessage(SessionMessageType::prompt, role, token_count, !b64_image.empty()),
      prompt_(prompt),
      b64_image_(b64_image)
    {}

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      if (b64_image_.empty()) {
        m["content"] = prompt_;
      } else {
        nlohmann::json msg = nlohmann::json::array();
        if (use_image()) {
          msg.push_back({{"type", "image_url"}, {"image_url", {{"url", b64_image_}}}});
          msg.push_back({{"type", "text"}, {"text", prompt_}});
        } else {
          msg.push_back({{"type", "text"}, {"text", prompt_ + "[image removed to reduce context size]"}});
        }
        m["content"] = msg;
      }
      return m;
    }

    std::string get_description() override {
      auto desc = SessionMessage::get_description();
      desc += ", prompt: " + prompt_ + ", has_image: " + (b64_image_.empty()? "N": "Y");
      return desc;
    }

    std::string prompt_;
    std::string b64_image_;
  };

  class SessionMessage_Response: public SessionMessage {
  public:
    SessionMessage_Response(const std::string role, const int token_count, const std::string &response):
      SessionMessage(SessionMessageType::response, role, token_count, false),
      response_(response)
    {}

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      m["content"] = response_;
      return m;
    }

    std::string get_description() override {
      auto desc = SessionMessage::get_description();
      desc += ", response: " + response_;
      return desc;
    }

    std::string response_;
  };

  class SessionMessage_ToolRequest: public SessionMessage {
  public:
    SessionMessage_ToolRequest(const std::string role, const int token_count, const std::string &tool_call_json):
      SessionMessage(SessionMessageType::tool_request, role, token_count, false),
      tool_call_json_(tool_call_json)
    {}

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      m["tool_calls"] = nlohmann::json::parse(tool_call_json_);
      return m;
    }

    std::string get_description() override {
      auto desc = SessionMessage::get_description();
      desc += ", tool_call_json: " + tool_call_json_;
      return desc;
    }

    std::string tool_call_json_;
  };

  class SessionMessage_ToolResult: public SessionMessage {
  public:
    SessionMessage_ToolResult(const std::string role, const int token_count, const std::string &name,
      const std::string &id, const std::string &tool_result_json, bool has_image):
      SessionMessage(SessionMessageType::tool_result, role, token_count, has_image),
      name_(name),
      id_(id),
      tool_result_json_(tool_result_json)
    {}

    nlohmann::json get_json(bool is_last) override {
      nlohmann::json m = SessionMessage::get_json(is_last);
      m["tool_call_id"] = id_;
      m["name"] = name_;

      if (has_image()) {
        // Content for image type is an object
        m["content"] = nlohmann::json::parse(tool_result_json_);

        // If the image should be purged, then remove the url and update the text field to indicate
        // the image was removed
        if (!use_image()) {
          if (m["content"].is_array()) {
            // The following handles multiple images in the result
            for (auto it = m["content"].begin(); it != m["content"].end(); ) {
              if (it->is_object() && it->value("type", "") == "image_url") {
                it = m["content"].erase(it);
              } else {
                ++it;
              }
            }

            for (auto &item: m["content"]) {
              if (item.contains("text")) {
                item["text"] += " [image removed to reduce context size]";
              }
            }
          }
        }
      } else {
        // Otherwise content contains json
        m["content"] = tool_result_json_;
      }          
      return m;
    }

    std::string get_description() override {
      auto desc = SessionMessage::get_description();
      desc += ", tool: " + name_ + ", tool_result_json: " + tool_result_json_;
      return desc;
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

  bool response_parse_error_{false};
  bool response_parse_error_tool_call_{false};

  TokenUsage token_usage_;
  TokenUsage token_usage_cur_msg_;

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
