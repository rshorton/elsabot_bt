#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include <ctime>
#include <iomanip>

#include "behaviortree_cpp/bt_factory.h"
#include "ai_session.hpp"
#include "ros_common.hpp"

class AIAction : public BT::StatefulActionNode {
 public:
  AIAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {
    node_ = ROSCommon::GetInstance()->GetNode();        
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("system_prompt"),
            BT::InputPort<bool>("new_session"),
            BT::InputPort<std::string>("prompt"),
            BT::OutputPort<std::string>("result")};
  }

  BT::NodeStatus onStart() {

    auto data_callback = [&](const std::string &data) {
      add_new_streaming_data(data);
    };

    bool new_session = false;
    getInput<bool>("new_session", new_session);

    if (new_session || !ai_session_) {
      std::string system_prompt = "You are a helpful assistant.";
      getInput<std::string>("system_prompt", system_prompt);

      ai_session_ = std::make_unique<AISession>(model_, max_context_size_, auth_token_,
                                                host_address_and_port_, resource_,
                                                timeout_ms_, system_prompt, node_->get_logger(),
                                                data_callback);
      if (!ai_session_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create an AI session object");
        return BT::NodeStatus::FAILURE;
      }                                                  
    }

    std::string prompt;
    getInput<std::string>("prompt", prompt);

    finish_on_next_update_ = false;
    new_sentence_ = std::string();

    auto tools_json = R"([
    {
      "type": "function",
      "function": {
        "name": "get_local_date_and_time",
        "description": "Gets the local date and time time in the form:  Y-M-D H:M:S"
      }
    },
    {
      "type": "function",
      "function": {
        "name": "scan_for_objects",
        "description": "scan for recognizable objects",
        "parameters": {
          "type": "object",
          "properties": {
              "object_type": {"type": "string", "description": "Type of object to detect"}
          },
          "required": ["object_type"]
        }
      }
    }    
    ])";

    ai_session_->user_prompt(prompt, tools_json);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() {
    if (finish_on_next_update_) {
      return BT::NodeStatus::SUCCESS;
    }

    // Output the latest sentence
    {
      std::lock_guard<std::mutex> guard(new_sentence_mutex_);
      if (!new_sentence_.empty()) {
        setOutput("result", new_sentence_);
        new_sentence_ = std::string();
        return BT::NodeStatus::RUNNING;
      }
    }

    std::string full_response;
    bool is_tool_call;
    AISession::Result result;
    auto finished = ai_session_->is_finished(full_response, is_tool_call, result);

    if (finished) {
      RCLCPP_DEBUG(node_->get_logger(), "AI request finished, result: %s", AISession::result_to_str(result).c_str());

      if (result == AISession::Result::cancelled ||
          result == AISession::Result::timeout ||
          result == AISession::Result::failed) {
        return BT::NodeStatus::FAILURE;

      } else {
        if (is_tool_call) {
          nlohmann::json tool_call = nlohmann::json::parse(full_response);

          std::string call_id = tool_call["id"];
          std::string name = tool_call["function"]["name"];
          nlohmann::json args = tool_call["function"]["arguments"];
          RCLCPP_INFO(node_->get_logger(), "Tool call, id: %s, name: %s, args: %s",
            call_id.c_str(), name.c_str(), args.dump().c_str());

          std::string result;
          if (name == "get_local_date_and_time") {
            nlohmann::json result_obj = {{"result", get_local_date_and_time()}};
            result = result_obj.dump();
          } else {
            // Dummy result for testing
            result = R"({"found": true})";
          }

          ai_session_->send_tool_result(call_id, name, result);
          return BT::NodeStatus::RUNNING;

        } else {
          // Run one more tick to allow the last of the response to be processed by other BT node(s)
          setOutput("result", streaming_data_buffer_);
          streaming_data_buffer_.clear();
          finish_on_next_update_ = true;
        }          
        return BT::NodeStatus::RUNNING;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() {
    if (ai_session_) {
      ai_session_->cancel();
    }
  }

 private:
  std::string get_local_date_and_time() {
    auto now = std::chrono::system_clock::now();

    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm local_tm; 
    localtime_r(&now_time_t, &local_tm);

    std::stringstream s;
    s << std::put_time(&local_tm, "%Y-%m-%d")
      << " "
      << std::put_time(&local_tm, "%H:%M:%S");

    return s.str();
}

  const std::size_t MIN_SENTENCE_LEN = 30;

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
          std::lock_guard<std::mutex> guard(new_sentence_mutex_);
          new_sentence_.append(sentence);
        }
        streaming_data_buffer_ = streaming_data_buffer_.substr(next_pos);
        break;
      }
    } while (pos > 0);
  }

  rclcpp::Node::SharedPtr node_;

#if defined(VLLM_GEMMA)  
  std::string model_{"cyankiwi/gemma-4-26B-A4B-it-AWQ-4bit"};
  std::string resource_{"/v1/chat/completions"};
  int max_context_size_{64000};
#else
  std::string resource_{"/completions"};
  int max_context_size_{4096};
#endif
  std::string auth_token_{"none"};

  std::string host_address_and_port_{"http://localhost:8000"};
  int timeout_ms_ = 8000;

  std::unique_ptr<AISession> ai_session_;

  std::string streaming_data_buffer_;

  std::mutex new_sentence_mutex_;
  std::string new_sentence_;

  bool finish_on_next_update_{false};
  
};
