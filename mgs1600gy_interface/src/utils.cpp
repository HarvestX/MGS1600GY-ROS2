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

#include "mgs1600gy_interface/utils.hpp"

namespace mgs1600gy_interface
{

void Utils::convertBGR(
  const std::array<float, 16> & in, cv::Mat * out,
  const float MIN, const float MAX, const bool FLIP)
{
  for (size_t i = 0; i < in.size(); ++i) {
    const float target = in.at(i);
    uint8_t val_S = 0, val_N = 0;
    if (target > 0.0) {
      // S pole
      if (target > MAX) {
        val_S = 255.0;
      } else {
        val_S =
          static_cast<uint8_t>(255.0 * target / MAX);
      }
    } else {
      // N pole
      if (target < MIN) {
        val_N = 255.0;
      } else {
        val_N =
          static_cast<uint8_t>(255.0 * target / MIN);
      }
    }
    int idx_assign = i;

    if (FLIP) {
      idx_assign = in.size() - i - 1;
    }
    out->at<cv::Vec3b>(0, idx_assign) = {val_S, 0, val_N};  // B G R
  }
}
}  // namespace mgs1600gy_interface
