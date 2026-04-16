#include <chrono>
#include <future>
#include <iostream>
#include <json.hpp>
#include <string>
#include <deque>

#include "behaviortree_cpp/bt_factory.h"
#include "http_request.hpp"
#include "ros_common.hpp"

#define VLLM_GEMMA

using namespace std::chrono_literals;

class AIChatAction : public BT::StatefulActionNode {
 public:
  AIChatAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {
    node_ = ROSCommon::GetInstance()->GetNode();        
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("system_prompt"),
            BT::InputPort<bool>("new_chat"),
            BT::InputPort<std::string>("prompt"),
            BT::InputPort<std::string>("pre_prompt"),
            BT::OutputPort<std::string>("result")};
  }

  BT::NodeStatus onStart() {
    bool new_chat = false;
    getInput<bool>("new_chat", new_chat);

    if (new_chat || !prev_chat_) {
      history_.clear();

      system_prompt_ =
          "You are a helpful assistant. Keep responses concise since the "
          "output will be rendered using text to speech. Use emojis when "
          "appropriate to express yourself.";
      getInput<std::string>("system_prompt", system_prompt_);
    }

    pre_prompt_ = std::string();
    std::string pre_prompt;
    getInput<std::string>("pre_prompt", pre_prompt_);

    std::string prompt;
    getInput<std::string>("prompt", prompt);

    prev_chat_ = true;
    finish_on_next_update_ = false;
    new_sentence_ = std::string();
    full_response_ = std::string();

    promise_ = std::move(promise<std::string>());
    future_ = std::move(promise_.get_future());

    auto completion_callback = [&](std::string data, CURLcode result) {
      RCLCPP_INFO(node_->get_logger(), "Completion_callback: %ld", data.length());

      std::string ret;
      if (result == CURLE_OK) {
        RCLCPP_INFO(node_->get_logger(), "Request successful");
        ret = "ok";
      } else if (result == CURLE_ABORTED_BY_CALLBACK) {
        RCLCPP_DEBUG(node_->get_logger(), "Model request was cancelled");
        ret = "aborted";
      } else if (result == CURLE_OPERATION_TIMEDOUT) {
        RCLCPP_ERROR(node_->get_logger(), "Model request timed-out");
        ret = "timeout";
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Model request failed: %s", curl_easy_strerror(result));
        ret = "failed";
      }
      promise_.set_value(ret);
    };

#if defined(VLLM_GEMMA)
    auto data_callback = [&](std::string data) {

      // vLLM sends data in Server-Sent Events (SSE) format: "data: {...}"
      std::cout << "data_callback: " << data << std::endl;

      size_t pos = 0;
      while ((pos = data.find("data: ", pos)) != std::string::npos) {
          pos += 6;
          size_t end = data.find("\n\n", pos);
          if (end == std::string::npos) break;

          std::string json_str = data.substr(pos, end - pos);
          if (json_str == "[DONE]") break;

          try {
              auto j = nlohmann::json::parse(json_str);
              if (j.contains("choices") && !j["choices"].empty()) {
                  std::string delta = j["choices"][0]["delta"]["content"];
                  add_new_streaming_data(delta);
              }
          } catch (...) {} 
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
          add_new_streaming_data(c);

        } catch (...) {
        }
        pos = end;
      }
    };
#endif

    std::for_each(history_.begin(), history_.end(), [](ChatMessage &item) {
      item.last_user_msg = false;
    });

    int num_tokens = fetch_token_count(prompt);
    history_.push_back({"user", prompt, num_tokens, true});
    current_tokens_ += num_tokens;

    RCLCPP_DEBUG(node_->get_logger(), "TOKENS, prompt: %d, total: %d", num_tokens, current_tokens_);

    while (current_tokens_ > (max_context_ - 1000) && !history_.empty()) {
      current_tokens_ -= history_.front().token_count;
      history_.pop_front();
    }

    RCLCPP_DEBUG(node_->get_logger(), "TOKENS, after purge:  total: %d", current_tokens_);

#if defined(VLLM_GEMMA)                              
    nlohmann::json request_data = {{"model", "cyankiwi/gemma-4-26B-A4B-it-AWQ-4bit"},
                                   {"stream", true},
                                   {"messages", format_prompt()}
                                  };

#else
    // We use the /completions endpoint for manual templating
    nlohmann::json request_data = {{"prompt", format_prompt()},
                                   {"stream", true},
                                   {"stop", {"</s>", "[INST]"}}
                                  };
#endif                            

    auto req_text = request_data.dump();

    RCLCPP_DEBUG(node_->get_logger(), "NEW REQUEST: %s", req_text.c_str());

    auto url = host_address_and_port_;
#if defined(VLLM_GEMMA)
    url += "/v1/chat/completions";
#else    
    url += "/completions";
#endif

    auto timeout = 8000;

    request_ = std::make_unique<HttpRequest>(true, url, timeout, std::string(), req_text,
                                             true, completion_callback, data_callback);
    request_->perform();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() {
    if (finish_on_next_update_) {
      return BT::NodeStatus::SUCCESS;
    }

    // Output the latest sentence
    {
      std::lock_guard<std::mutex> guard(new_sentence_mutex);
      if (!new_sentence_.empty()) {
        setOutput("result", new_sentence_);
        new_sentence_ = std::string();
        return BT::NodeStatus::RUNNING;
      }
    }

    if (!request_) {
      return BT::NodeStatus::FAILURE;
    }

    auto status = future_.wait_for(0ms);

    if (status == std::future_status::ready) {
      auto result = future_.get();
      RCLCPP_DEBUG(node_->get_logger(), "Future is ready, result: %s", result.c_str());

      if (result == "aborted" || result == "timeout" || result == "failed") {
        return BT::NodeStatus::FAILURE;
      } else {
        // Run one more tick to allow the last of the response to be processed
        setOutput("result", streaming_data_buffer_);
        streaming_data_buffer_.clear();
        finish_on_next_update_ = true;

        int num_tokens = fetch_token_count(full_response_);
        history_.push_back({"assistant", full_response_, num_tokens, false});
        current_tokens_ += num_tokens;

        return BT::NodeStatus::RUNNING;
      }
    } else if (status == std::future_status::timeout) {
      RCLCPP_DEBUG(node_->get_logger(), "Future is not ready yet (timeout).");
    } else if (status == std::future_status::deferred) {
      RCLCPP_DEBUG(node_->get_logger(), "Future task is deferred (not started yet).");
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() {
    if (request_) {
      request_->cancel();
    }
  }

 private:
  const std::size_t MIN_SENTENCE_LEN = 30;

  struct ChatMessage {
    std::string role;
    std::string content;
    int token_count;
    bool last_user_msg;
  };

  int fetch_token_count(const std::string& text) {
#if defined(VLLM_GEMMA)
    int count = (int)text.length() / 4;

#else
    nlohmann::json body = {{"content", text}};
    // Default count to length/4 if cannot request
    int count = (int)text.length() / 4;

    auto completion_callback = [&](std::string data, CURLcode result) {
      RCLCPP_DEBUG(node_->get_logger(), "Completion_callback (token count): %s", data.c_str());

      if (result == CURLE_OK) {
        RCLCPP_DEBUG(node_->get_logger(), "Token count request successful. Data length: %ld bytes", data.length());
        count = (int)nlohmann::json::parse(data)["tokens"].size();
      } else if (result == CURLE_OPERATION_TIMEDOUT) {
        RCLCPP_ERROR(node_->get_logger(), "Token count request timed-out.");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Token count request failed: %s", curl_easy_strerror(result));
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
  nlohmann::json format_prompt() {

    nlohmann::json prompt = nlohmann::json::array();

    prompt.push_back({{"role", "system"}, {"content", system_prompt_}});

    for (const auto& msg : history_) {
        prompt.push_back({{"role", msg.role}, {"content", msg.content}});
    }

    return prompt;
  }
#else
  // Manual Llama 2 Template Builder
  std::string format_prompt() {
    std::string prompt =
        "<s>[INST] <<SYS>>\n" + system_prompt_ + "\n<</SYS>>\n\n";
    
    bool first_user = true;

    for (const auto& msg : history_) {
      if (msg.role == "user") {
        if (!first_user) {
          prompt += "<s>[INST] ";
        }

        // Include optional 'extra' system prompt before new prompt
        if (msg.last_user_msg && !pre_prompt_.empty()) {
          prompt += "<<SYS>>\n" + pre_prompt_ + "\n<</SYS>>\n\n";
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

  void add_new_streaming_data(const std::string& delta) {
    streaming_data_buffer_.append(delta);

    if (streaming_data_buffer_.empty()) {
      return;
    }

    // Remove characters that should be silent
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
          std::lock_guard<std::mutex> guard(new_sentence_mutex);
          new_sentence_.append(sentence);
        }
        streaming_data_buffer_ = streaming_data_buffer_.substr(next_pos);
        break;
      }
    } while (pos > 0);
  }

  rclcpp::Node::SharedPtr node_;

  std::string host_address_and_port_{"http://localhost:8000"};

  std::promise<std::string> promise_;
  std::future<std::string> future_;

  std::unique_ptr<HttpRequest> request_;

  std::string streaming_data_buffer_;

  std::mutex new_sentence_mutex;
  std::string new_sentence_;

  bool finish_on_next_update_{false};

  bool prev_chat_{false};
  std::string system_prompt_;
  std::deque<ChatMessage> history_;
  // fix - add param
  int max_context_{4096};
  int current_tokens_{0};

  std::string pre_prompt_;

  std::string full_response_;
};
