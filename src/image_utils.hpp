#pragma once

#include <string>

namespace image_utils
{
bool save_image(const std::string &dir, const std::string &b64_image, std::string &saved_to_fpath);
}