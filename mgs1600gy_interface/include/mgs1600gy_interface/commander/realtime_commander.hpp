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
#include <memory>
#include <array>
#include <rclcpp/rclcpp.hpp>


#include "mgs1600gy_interface/packet_handler.hpp"

namespace mgs1600gy_interface
{
using namespace std::chrono_literals;
class RealtimeCommander
{
public:
  enum class MODE
  {
    NORMAL,
    QUERY,
  };

private:
  std::shared_ptr<PacketHandler> packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

public:
  RealtimeCommander() = delete;
  explicit RealtimeCommander(
    std::shared_ptr<PacketHandler>,
    const rclcpp::Duration &);

  RESPONSE_STATE readMZ(std::array<float, 16> &, const MODE) const noexcept;
  RESPONSE_STATE readANG(std::array<float, 3> &, const MODE) const noexcept;

  RESPONSE_STATE startQuery(const uint32_t every_ms) const noexcept;
  RESPONSE_STATE clearQuery() const noexcept;

  RESPONSE_STATE setAngZero(const int) const noexcept;

private:
  static const rclcpp::Logger getLogger() noexcept;
  bool waitForResponse(
    const PacketPool::PACKET_TYPE &, std::string &) const noexcept;

};
}  // namespace mgs1600gy_interface
