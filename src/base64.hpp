#pragma once

#include <string>
#include <vector>

std::string base64_encode(const std::vector<unsigned char>& data);
std::vector<unsigned char> base64_decode(const std::string& in);