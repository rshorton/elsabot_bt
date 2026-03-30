#include <chrono>
#include <future>
#include <iostream>
#include <json.hpp>
#include <string>

#include "async_http_request.hpp"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

namespace {
const std::size_t MIN_SENTENCE_LEN = 30;
}

class AIModelRequest : public BT::StatefulActionNode {
 public:
  AIModelRequest(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("prompt"),
            BT::OutputPort<std::string>("result")};
  }

  BT::NodeStatus onStart() {
    std::string prompt;
    getInput<std::string>("prompt", prompt);

    use_streaming_ = false;
    finish_on_next_update_ = false;
    new_sentence_ = std::string();

    promise_ = std::move(promise<std::string>());
    future_ = std::move(promise_.get_future());

    auto completion_callback = [&](std::string data, CURLcode result) {
      std::string ret;
      if (result == CURLE_OK) {
        std::cout << "Request successful. Data length: " << data.length()
                  << " bytes\n";
        // std::cout << "Data: " << data << "\n";
        ret = data;
      } else if (result == CURLE_ABORTED_BY_CALLBACK) {
        std::cout << "Request was cancelled.\n";
        ret = "aborted";
      } else if (result == CURLE_OPERATION_TIMEDOUT) {
        std::cout << "Request timed-out.\n";
        ret = "timeout";
      } else {
        std::cerr << "Request failed: " << curl_easy_strerror(result)
                  << std::endl;
        ret = "failed";
      }
      promise_.set_value(ret);
    };

    auto data_callback = [&](std::string data) {
      // std::cout << "data_callback: " << data << std::endl;
      //  Example:  data:
      //  {"type":"response.output_text.delta","item_id":"msg_b3e4f043-dcde-45f0-8e4e-489e3cb1a99d","output_index":0,"content_index":0,"delta":"Still"}

      const std::string data_prefix = "data: ";
      if (data.compare(0, data_prefix.length(), data_prefix) != 0) {
        return;
      }

      try {
        nlohmann::json j =
            nlohmann::json::parse(data.substr(data_prefix.length()));

        auto t = j["type"].get<std::string>();
        if (t != "response.output_text.delta") {
          return;
        }

        auto delta = j["delta"].get<std::string>();
        std::cout << "Text Delta: " << delta << std::endl;

        add_new_streaming_data(delta);
      } catch (...) {
        std::cout << "Failed to parse data callback json" << std::endl;
      }
    };

    // Fix replace with env var;
    auto url = "http://localhost:18789/v1/responses";
    auto auth_token = "16d500aaf52f183d113206c8c03d8fac79341b1b775c8714";
    auto timeout = 90000;

    nlohmann::json req = {{"model", "openclaw:main"},
                          {"input", prompt},
                          {"stream", use_streaming_},
                          {"user", "user:scott"},
                          {"instructions",
                           "Keep the response concise to it can be efficiently "
                           "rendered using TTS."},
                          {"max_output_tokens", 1024}};

    request_ = std::make_unique<AsyncHttpRequest>(
        url, timeout, auth_token, req.dump(), use_streaming_,
        completion_callback, data_callback);
    request_->perform();

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() {
    if (finish_on_next_update_) {
      return BT::NodeStatus::SUCCESS;
    }

    // Output the latest sentence if in streaming mode
    if (use_streaming_) {
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
      std::cout << "Future is ready, result is available." << std::endl;
      auto result = future_.get();
      std::cout << "Result: " << result << std::endl;
      if (result == "aborted" || result == "timeout" || result == "failed") {
        return BT::NodeStatus::FAILURE;
      } else {
        if (use_streaming_) {
          // Run one more tick to allow the last of the response to be processed
          setOutput("result", streaming_data_buffer_);
          streaming_data_buffer_.clear();
          finish_on_next_update_ = true;
          return BT::NodeStatus::RUNNING;
        } else {
          std::string ret;
          if (parse_response(result, ret)) {
            setOutput("result", ret);
            finish_on_next_update_ = true;
            return BT::NodeStatus::RUNNING;
          }
          std::cout << "Failed to parse model result json" << std::endl;
          return BT::NodeStatus::FAILURE;
        }
      }
    } else if (status == std::future_status::timeout) {
      //std::cout << "Future is not ready yet (timeout)." << std::endl;
    } else if (status == std::future_status::deferred) {
      std::cout << "Future task is deferred (not started yet)." << std::endl;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() {
    if (request_) {
      request_->cancel();
    }
  }

 private:
  /* Example from openclaw
    {"id":"resp_f0456f65-909b-40ed-8657-c2eaa2e13703",
    "object":"response",
    "created_at":1773420798,
    "status":"completed",
    "model":"openclaw:main",
    "output":[
      {"type":"message","id":"msg_c0c28c95-812e-458a-996d-2d3de075f786","role":"assistant",
        "content":
        [
            {"type":"output_text","text":"Hey! I just woke up and I'm still
    figuring that out. Let me check who you are first, and then we can figure
    out what to call me.\n\nHow about you start by telling me who you are and
    how you'd like to be addressed? Then we can work out my identity
    together.\n\nAre you the human I'm here to help? What should I call you?"}
        ],
        "status":"completed"
      }
    ],
    "usage":{"input_tokens":0,"output_tokens":0,"total_tokens":0}}
  */
  bool parse_response(std::string result, std::string& ret) const {
    try {
      nlohmann::json j = nlohmann::json::parse(result);
      std::cout << std::setw(4) << j << "\n\n";

      auto object = j["object"].get<std::string>();
      if (object != "response") {
        std::cerr << "Unexpected model response: " << object << std::endl;
        return false;
      }

      std::cout << "Model response object: " << object << std::endl;

      auto status = j["status"].get<std::string>();
      if (status != "completed") {
        std::cerr << "Unexpected model status: " << status << std::endl;
        return false;
      }

      std::cout << "Got model status: " << status << std::endl;

      for (const auto& out_item : j["output"]) {
        if (out_item.find("content") == out_item.end()) {
          std::cerr << "Did find content in model output" << std::endl;
          return false;
        }

        for (const auto& content_item : out_item["content"]) {
          if (content_item.find("text") == content_item.end()) {
            std::cerr << "Did find text in content model output" << std::endl;
            return false;
          }
          ret = content_item["text"].get<std::string>();
          return true;
        }
      }
    } catch (...) {
      std::cout << "Failed to parse model result json" << std::endl;
    }
    return false;
  }

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
    std::string separators = ".?!";
    size_t pos = len;
    do {
      pos = streaming_data_buffer_.find_last_of(".?!", pos - 1);
      if (pos == std::string::npos) {
        break;
      }

      auto next_pos = pos + 1;
      if (next_pos < len && streaming_data_buffer_[next_pos] == ' ') {
        auto sentence = streaming_data_buffer_.substr(0, next_pos);
        std::cout << "New Sentence: " << sentence << std::endl;
        {
          std::lock_guard<std::mutex> guard(new_sentence_mutex);
          new_sentence_.append(sentence);
        }
        streaming_data_buffer_ = streaming_data_buffer_.substr(next_pos);
        break;
      }
    } while (pos > 0);
  }

  std::promise<std::string> promise_;
  std::future<std::string> future_;

  std::unique_ptr<AsyncHttpRequest> request_;

  std::string streaming_data_buffer_;

  std::mutex new_sentence_mutex;
  std::string new_sentence_;

  bool use_streaming_{false};
  bool finish_on_next_update_{false};
};
