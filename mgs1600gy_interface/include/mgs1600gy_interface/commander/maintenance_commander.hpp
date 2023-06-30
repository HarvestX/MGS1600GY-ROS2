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
class MaintenanceCommander
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(MaintenanceCommander)

private:
  PacketHandler::SharedPtr packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const std::chrono::nanoseconds TIMEOUT_;

public:
  MaintenanceCommander() = delete;
  explicit MaintenanceCommander(
    PacketHandler::SharedPtr, const std::chrono::nanoseconds &);

  RESPONSE_STATE writeZERO() const noexcept;
  RESPONSE_STATE writeGZER() const noexcept;

  RESPONSE_STATE writeCLSAV() const noexcept;

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace mgs1600gy_interface
