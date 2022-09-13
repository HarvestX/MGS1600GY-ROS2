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

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mgs1600gy_interface/packet_handler.hpp"

namespace mgs1600gy_interface
{
using namespace std::chrono_literals;
// TODO(m12watanabe1a)
// Not implemented yet
class MaintenanceCommander
{
private:
  std::shared_ptr<PacketHandler> packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

public:
  MaintenanceCommander() = delete;
  explicit MaintenanceCommander(
    std::shared_ptr<PacketHandler>,
    const rclcpp::Duration &);

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace mgs1600gy_interface
