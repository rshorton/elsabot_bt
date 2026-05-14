#include "image_utils.hpp"

#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ctime>
#include <regex>
#include <chrono>

#include "base64.hpp"

using namespace std::chrono;

namespace image_utils
{

std::string get_ts_filename(const std::string &dir, const std::string& extension) {
    auto now = system_clock::now();

    // Extract milliseconds
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    auto time = system_clock::to_time_t(now);
    auto tm = *std::localtime(&time);
    
    std::ostringstream oss;
    oss << dir << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
               << '_' << std::setfill('0') << std::setw(3) << ms.count()
               << "." <<  extension;
    return oss.str();
}

// Save b64 image to file using timestamp and return file name
bool save_image(const std::string &dir, const std::string &b64_image, std::string &saved_to_fpath) {

    auto found = b64_image.find(",");
    if (found == std::string::npos) {
        return false;
    }
    auto header = b64_image.substr(0, found);
    auto b64_data = b64_image.substr(found + 1);

    // Don't use this to extract the base64 image data since that will likely cause a crash
    // due to stack overflow
    std::regex ws_re("data:image/([a-zA-Z0-9]+);base64");
    std::smatch match;

    if (!std::regex_search(header, match, ws_re)) {
        std::cerr << "Failed to save image, missing b64 prefix" << std::endl;
        return false;
    }
    std::string imageType = match[1];

    auto image_bytes = base64_decode(b64_data);

    saved_to_fpath = get_ts_filename(dir, imageType);
    std::ofstream out_file(saved_to_fpath, std::ios::binary);
    if (out_file.is_open()) {
        out_file.write(reinterpret_cast<const char*>(image_bytes.data()), image_bytes.size());
        out_file.close();
    } else {
        std::cerr << "Failed to save image to: " << saved_to_fpath << std::endl;
        return false;
    }
    return true;
}

} // namespace image_utils