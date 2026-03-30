/* Created using Google Ai Overview feature */

#ifndef ASYNC_HTTP_REQUEST_H
#define ASYNC_HTTP_REQUEST_H

#include <curl/curl.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class AsyncHttpRequest {
 public:
  // Callback type for when the request completes
  using CallbackDone =
      std::function<void(std::string response_data, CURLcode result)>;
  using CallbackData = std::function<void(std::string response_data)>;

  /**
   * @brief Construct a new Async Http Request object
   * @param url The URL to request
   * @param timeout_ms Timeout in milliseconds (0 for no timeout)
   * @param callback The function to call upon completion
   */
  AsyncHttpRequest(const std::string& url, long timeout_ms,
                   std::string auth_token, std::string data, bool use_streaming,
                   CallbackDone callback_done, CallbackData callback_data);
  ~AsyncHttpRequest();

  // Starts the asynchronous request in its own thread
  void perform();
  // Attempts to cancel the ongoing request
  void cancel();

  static bool inited_;
  static void startup_init() {
    if (!inited_) {
      curl_global_init(CURL_GLOBAL_DEFAULT);
      inited_ = true;
    }
  }

  static void shutdown_deinit() {
    if (inited_) {
      curl_global_cleanup();
      inited_ = false;
    }
  }

 private:
  // The main loop running in the request thread
  void requestLoop();
  // Callback function to write received data
  static size_t WriteCallback(void* contents, size_t size, size_t nmemb,
                              void* userp);

  std::string url_;
  long timeout_ms_;
  std::string auth_token_;
  std::string req_data_;

  bool use_streaming_;
  CallbackDone callback_done_;
  CallbackData callback_data_;
  std::string response_data_;
  std::thread request_thread_;
  std::atomic<bool> is_cancelled_{false};
  CURL* easy_handle_ = nullptr;
  CURLM* multi_handle_ = nullptr;
  std::mutex mtx_;
  std::condition_variable cv_;
};

#endif  // ASYNC_HTTP_REQUEST_H
