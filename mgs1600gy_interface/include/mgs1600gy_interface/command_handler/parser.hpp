// Copyright 2022 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#pragma once

#include <algorithm>
#include <sstream>
#include <vector>
#include <string>
#include <array>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

namespace mgs1600gy_interface
{

template<typename T, size_t sizeOfArray>
class Parser
{
private:
  const rclcpp::Logger logger_;

public:
  Parser() = delete;
  explicit Parser(const std::string && logger_name)
  : logger_(rclcpp::get_logger(logger_name))
  {}

  bool parse(
    const std::string & input,
    std::array<int, sizeOfArray> & data)
  {
    // Check string integrity
    // '=' should exist 1
    // ':' should exist (sensor_num - 1)
    const int equal_cnt =
      std::count(input.begin(), input.end(), '=');
    if (equal_cnt != 1) {
      RCLCPP_ERROR(
        this->logger_,
        "Invalid number of '=' %d, expected 1", equal_cnt);
      return false;
    }
    const int sep_cnt =
      std::count(input.begin(), input.end(), ':');
    if (sep_cnt != data.size() - 1) {
      RCLCPP_ERROR(
        this->logger_,
        "Invalid number of ':' %d, expected %ld",
        sep_cnt, data.size() - 1);
      return false;
    }

    std::string token;
    token.resize(10);

    std::stringstream ss(input);
    // take first value
    std::getline(ss, token, '=');
    token.clear();

    for (size_t i = 0; i < data.size(); ++i) {
      std::getline(ss, token, ':');
      if (token.empty()) {
        RCLCPP_ERROR(
          this->logger_,
          "Empty token");
        return false;
      }
      data.at(i) = std::stoi(token);
      token.clear();
    }
    return true;
  }

  void showParsedResult(const std::array<int, sizeOfArray> & data)
  {
    std::stringstream ss;
    for (auto val : data) {
      ss << val << " ";
    }
    RCLCPP_INFO(
      this->logger_,
      ss.str().c_str());
  }
};
}  // namespace mgs1600gy_interface
