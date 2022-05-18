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
#include "mgs1600gy_interface/port_handler.hpp"
#include <rclcpp/rclcpp.hpp>

namespace mgs1600gy_interface
{
class Prettier
{
private:
  const std::string realtime_write_prefix_ = "!";
  const std::string maintenance_write_prefix_ = "%";
  const std::string prefix_;
  const std::string suffix_;

public:
  Prettier() = delete;
  explicit Prettier(const std::string &);
  std::string exec(const std::string &&) const noexcept;
  std::string exec(const std::string &&, const int) const noexcept;
  std::string exec(const std::string &&, const int, const int) const noexcept;
};
}  // namespace mgs1600gy_interface
