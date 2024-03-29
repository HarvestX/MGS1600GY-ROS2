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
#include <rclcpp/rclcpp.hpp>


namespace mgs1600gy_interface
{

class Utils
{
public:
  static void convertBGR(const std::array<float, 16> &, cv::Mat *, const float, const float);
  static void convertBGR(
    const std::array<float, 16> &, std::array<uint8_t, 16 * 3> &, const float,
    const float);
};
}  // namespace mgs1600gy_interface
