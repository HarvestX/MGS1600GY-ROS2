// Copyright 2022 HarvestX Inc
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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mgs1600gy_interface/mgs1600gy_interface.hpp>

namespace mgs1600gy_tool
{
class GyroCalibrator : public rclcpp::Node
{
private:
  std::unique_ptr<mgs1600gy_interface::Mgs1600gyInterface> interface_;

public:
  GyroCalibrator() = delete;
  explicit GyroCalibrator(const rclcpp::NodeOptions &);
  ~GyroCalibrator();

  void calibrate();
};
}  // namespace mgs1600gy_tool
