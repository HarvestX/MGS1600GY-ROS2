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

#include <queue>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>

namespace mgs1600gy_interface
{

class PacketPool
{
public:
  enum class PACKET_TYPE
  {
    MZ,
    ANG,
    END_PACKET_TYPE
  };

private:
  std::map<PACKET_TYPE, std::queue<std::string>> queue_map_;

public:
  PacketPool();
  ~PacketPool();

  void clear();
  void enqueue(const std::string &);
  bool takePacket(const PACKET_TYPE &, std::string &);
  static bool parseResponse(const std::string &, std::vector<float> &) noexcept;
  static std::string packetTypeToString(const PACKET_TYPE &) noexcept;

private:
  static const rclcpp::Logger getLogger() noexcept;
  static const std::string fixEscapeSequence(const std::string &);
};
}  // namespace mgs1600gy_interface
