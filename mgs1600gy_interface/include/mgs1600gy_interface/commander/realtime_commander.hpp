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

#include <string>
#include <array>
#include <rclcpp/rclcpp.hpp>

namespace mgs1600gy_interface
{
class RealtimeCommander
{
public:
  RealtimeCommander();
  void readMZ(std::array<int, 16> &);
  void readMZ(const std::string &, std::array<int, 16> &);

  void readANG(std::array<double, 3> &);
  void readANG(const std::string &, std::array<double, 3> &);

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace mgs1600gy_interface
