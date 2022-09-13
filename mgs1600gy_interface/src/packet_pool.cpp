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


#include "mgs1600gy_interface/packet_pool.hpp"


namespace mgs1600gy_interface
{
PacketPool::PacketPool()
{
  this->clear();
}

PacketPool::~PacketPool()
{
  this->clear();
}

void PacketPool::clear()
{
  for (size_t i = 0;
    i < static_cast<size_t>(PACKET_TYPE::END_PACKET_TYPE);
    ++i)
  {
    auto queue = this->queue_map_[static_cast<PACKET_TYPE>(i)];
    while (!queue.empty()) {
      queue.pop();
    }
  }
}

void PacketPool::enqueue(const std::string & in_packet)
{
  static std::string previous_chunk = "";
  std::string chunk = previous_chunk + in_packet;
  previous_chunk.clear();

  std::vector<std::string> packet_candidates;

  std::string item = "";
  for (char ch : chunk) {
    item += ch;
    if (ch != '\r') {
      continue;
    }
    if (item.empty()) {
      continue;
    }

    if (item.rfind("MZ=", 0) == 0) {
      RCLCPP_DEBUG(
        this->getLogger(),
        "MZ packet stocked : %s",
        item.c_str());
      this->queue_map_[PACKET_TYPE::MZ].push(item);
    } else if (item.rfind("ANG=", 0) == 0) {
      RCLCPP_DEBUG(
        this->getLogger(),
        "ANG packet stocked : %s",
        item.c_str());
      this->queue_map_[PACKET_TYPE::ANG].push(item);
    } else if (
      item.at(0) == '?' ||
      item.at(0) == '#' ||
      item.at(0) == '\r')
    {
      // Skip
      // This is echo back from previous packet
    } else {
      RCLCPP_WARN(
        this->getLogger(),
        "Command like packet [%s] given. Ignored.",
        PacketPool::fixEscapeSequence(item).c_str());
    }


    item.clear();
  }


  // Stock previous result
  if (!item.empty()) {
    previous_chunk = item;
  }
}

bool PacketPool::takePacket(
  const PACKET_TYPE & target,
  std::string & packet)
{
  bool res = false;
  packet.clear();
  if (!this->queue_map_[target].empty()) {
    packet = this->queue_map_[target].front();
    this->queue_map_[target].pop();
    res = true;
  }
  return res;
}

bool PacketPool::parseResponse(
  const std::string & response,
  std::vector<float> & out) noexcept
{
  bool scanning_started = false;
  std::string val_candidate;
  out.clear();
  for (char ch : response) {
    if (ch == '=') {
      scanning_started = true;
      val_candidate.clear();
      continue;
    }
    if (!scanning_started) {
      continue;
    }
    if (ch == '\r' || ch == ':') {
      try {
        out.emplace_back(
          std::stof(val_candidate));
      } catch (std::invalid_argument & e) {
        RCLCPP_ERROR(
          PacketPool::getLogger(),
          "%s", e.what());
        out.clear();
        return false;
      }
      val_candidate.clear();
      continue;
    }

    val_candidate += ch;
  }

  return true;
}


std::string PacketPool::packetTypeToString(
  const PACKET_TYPE & packet_type) noexcept
{
  switch (packet_type) {
    case PACKET_TYPE::MZ:
      return "MZ";
    case PACKET_TYPE::ANG:
      return "ANG";
    default:
      // Do nothing
      break;
  }
  return "";
}


const rclcpp::Logger PacketPool::getLogger() noexcept
{
  return rclcpp::get_logger("PacketPool");
}

const std::string PacketPool::fixEscapeSequence(const std::string & in)
{
  std::string out;
  for (auto c : in) {
    switch (c) {
      case '\r':
        out += "\\r";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\0':
        out += "\\0";
        break;
      default:
        out += c;
    }
  }
  return out;
}
}  // namespace mgs1600gy_interface
