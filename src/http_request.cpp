#include "http_request.hpp"

#include <chrono>

bool HttpRequest::inited_ = false;

HttpRequest::HttpRequest(bool async, const std::string& url, long timeout_ms,
                         std::string auth_token, std::string req_data,
                         bool use_streaming, CallbackDone callback_done,
                         CallbackData callback_data)
    : async_(async),
      url_(url),
      timeout_ms_(timeout_ms),
      auth_token_(auth_token),
      req_data_(std::move(req_data)),
      use_streaming_(use_streaming),
      callback_done_(std::move(callback_done)),
      callback_data_(std::move(callback_data)) {
  std::cout << "HttpRequest 0\n";

  startup_init();

  std::cout << "HttpRequest 1\n";
}

HttpRequest::~HttpRequest() {
  if (request_thread_.joinable()) {
    request_thread_.join();
  }
  // curl_global_cleanup() should be called at application exit
}

void HttpRequest::perform() {
  if (async_) {
    request_thread_ = std::thread(&HttpRequest::requestLoop, this);
  } else {
    requestLoop();
  }    
}

void HttpRequest::cancel() {
  is_cancelled_.store(true);
  // Might need a mechanism to wake up the select/poll call in requestLoop if
  // it's blocking For this simple thread-based approach, setting the flag is
  // enough for the loop to check.
}

size_t HttpRequest::WriteCallback(void* contents, size_t size,
                                       size_t nmemb, void* userp) {
  size_t total_size = size * nmemb;

  HttpRequest* self = static_cast<HttpRequest*>(userp);
  if (!self) {
    return total_size;
  }

  std::string data = std::string(static_cast<char*>(contents), total_size);
  // std::cout << "Write Callback: " << std::endl << data << std::endl << "Write
  // CB End" << std::endl;

  self->response_data_.append(data);

  if (self->use_streaming_ && self->callback_data_) {
    self->callback_data_(data);
  }
  return total_size;
}

void HttpRequest::requestLoop() {
  CURLMcode mc;
  int still_running = 0;
  CURLcode res = CURLE_OK;

  easy_handle_ = curl_easy_init();
  multi_handle_ = curl_multi_init();

  if (!easy_handle_ || !multi_handle_) {
    callback_done_("", CURLE_FAILED_INIT);
    return;
  }

  // Set easy handle options
  curl_easy_setopt(easy_handle_, CURLOPT_URL, url_.c_str());

  struct curl_slist* headers = NULL;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  // Add the Authorization header with the Bearer token
  if (auth_token_.length() > 0) {
    std::string auth_header = "Authorization: Bearer " + auth_token_;
    headers = curl_slist_append(headers, auth_header.c_str());
  }

  std::string accept_header = "Accept: application/json";
  headers =  curl_slist_append(headers, accept_header.c_str());

  // Temp workaround for openclaw 3/28 issue: https://github.com/openclaw/openclaw/issues/46997
  //std::string hdr = "x-openclaw-scopes: operator.read,operator.write";
  //headers = curl_slist_append(headers, hdr.c_str());

  curl_easy_setopt(easy_handle_, CURLOPT_HTTPHEADER, headers);

  curl_easy_setopt(easy_handle_, CURLOPT_POSTFIELDS, req_data_.c_str());
  curl_easy_setopt(easy_handle_, CURLOPT_POSTFIELDSIZE,
                   (long)req_data_.length());

  curl_easy_setopt(easy_handle_, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(easy_handle_, CURLOPT_WRITEDATA, this);
  if (timeout_ms_ > 0) {
    // Set total timeout for the operation
    curl_easy_setopt(easy_handle_, CURLOPT_TIMEOUT_MS, timeout_ms_);
  }

  // Add easy handle to the multi stack
  curl_multi_add_handle(multi_handle_, easy_handle_);

  do {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(10));  // Prevent busy looping
    mc = curl_multi_perform(multi_handle_, &still_running);

    if (is_cancelled_.load()) {
      // Abort the transfer
      break;
    }

    if (still_running) {
      // Wait for activity (socket or timeout)
      mc = curl_multi_poll(multi_handle_, NULL, 0, 1000, NULL);
      if (mc) break;  // Error in polling
    }

  } while (still_running);

  if (is_cancelled_.load()) {
    res = CURLE_ABORTED_BY_CALLBACK;  // Or a custom error code
  } else {
    // Check the result of the completed transfer
    CURLMsg* msg;
    int msgs_left;
    while ((msg = curl_multi_info_read(multi_handle_, &msgs_left))) {
      if (msg->msg == CURLMSG_DONE) {
        res = msg->data.result;
        break;
      }
    }
  }

  // Cleanup
  curl_multi_remove_handle(multi_handle_, easy_handle_);
  curl_easy_cleanup(easy_handle_);
  curl_multi_cleanup(multi_handle_);

  // Call the user callback
  callback_done_(response_data_, res);
}
