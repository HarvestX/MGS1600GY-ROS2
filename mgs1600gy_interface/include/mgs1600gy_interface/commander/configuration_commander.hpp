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

#include <rclcpp/rclcpp.hpp>

#include "mgs1600gy_interface/packet_handler.hpp"


namespace mgs1600gy_interface
{
using namespace std::chrono_literals;  // NOLINT
// TODO(m12watanabe1a)
// Not implemented yet
class ConfigurationCommander
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(ConfigurationCommander)

private:
  PacketHandler::SharedPtr packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const std::chrono::nanoseconds TIMEOUT_;

public:
  ConfigurationCommander() = delete;
  explicit ConfigurationCommander(PacketHandler::SharedPtr, const std::chrono::nanoseconds &);

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace mgs1600gy_interface
