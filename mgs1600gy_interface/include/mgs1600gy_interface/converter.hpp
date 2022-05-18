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


#include <opencv2/opencv.hpp>

namespace mgs1600gy_interface
{
template<typename T, size_t sizeOfArray>
class Converter
{
public:
  const int DATA_MIN, DATA_MAX;
  const int KERNEL_SIZE = 11;

public:
  Converter() = delete;
  explicit Converter(const int min, const int max)
  : DATA_MIN(min), DATA_MAX(max)
  {
    if (min >= max) {
      throw std::runtime_error("MIN is greater than MAX");
    }
  }

  cv::Mat yieldBaseCvMat()
  {
    cv::Mat ret(1, sizeOfArray, CV_8UC1);
    return ret;
  }

  cv::Mat yieldBaseBlurCvMat()
  {
    cv::Mat ret(1, sizeOfArray * 10, CV_8UC1);
    return ret;
  }

  void convert(const std::array<T, sizeOfArray> & in_data, cv::Mat & out_data)
  {
    for (size_t i = 0; i < sizeOfArray; ++i) {
      uint8_t val_assign;
      if (in_data.at(i) < DATA_MIN) {
        val_assign = 0;
      } else if (in_data.at(i) > DATA_MAX) {
        val_assign = 255;
      } else {
        val_assign =
          static_cast<uint8_t>(255.0 * in_data.at(i) / DATA_MAX);
      }
      // Flip output
      val_assign = 255 - val_assign;
      out_data.at<uint8_t>(0, i) = val_assign;
    }
  }

  void convertBlur(
    const std::array<T, sizeOfArray> & in_data, cv::Mat & out_data)
  {
    cv::Mat tmp = out_data;
    for (size_t i = 0; i < sizeOfArray; ++i) {
      uint8_t val_assign;
      if (in_data.at(i) < DATA_MIN) {
        val_assign = 0;
      } else if (in_data.at(i) > DATA_MAX) {
        val_assign = 255;
      } else {
        val_assign =
          static_cast<uint8_t>(255.0 * in_data.at(i) / DATA_MAX);
      }
      // Flip output
      val_assign = 255 - val_assign;
      for (size_t j = 0; j < 10; ++j) {
        tmp.at<uint8_t>(0, i * 10 + j) = val_assign;
      }
    }
    cv::GaussianBlur(tmp, out_data, cv::Size(this->KERNEL_SIZE, 1), 0);
  }
};
}  // namespace mgs1600gy_interface
