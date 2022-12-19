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

#include "mgs1600gy_interface/commander/realtime_commander.hpp"


namespace  mgs1600gy_interface
{
RealtimeCommander::RealtimeCommander(
  PacketHandler::SharedPtr packet_handler,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
  const std::chrono::nanoseconds & timeout)
: packet_handler_(packet_handler),
  logging_interface_(logger),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(timeout)
{}


RESPONSE_STATE RealtimeCommander::readMZ(
  std::array<float, 16> & _out,
  const MODE mode) const noexcept
{
  if (mode == MODE::NORMAL) {
    static const char write_buf[] = "?MZ\r";
    this->packet_handler_->writePort(write_buf, sizeof(write_buf));
  }

  std::string response;
  if (!this->waitForResponse(PacketPool::PACKET_TYPE::MZ, response)) {
    return RESPONSE_STATE::ERROR_NO_RESPONSE;
  }

  std::vector<float> res_vals;
  if (!PacketPool::parseResponse(response, res_vals)) {
    return RESPONSE_STATE::ERROR_PARSE_FAILED;
  }

  if (res_vals.size() != _out.size()) {
    return RESPONSE_STATE::ERROR_PARSE_RESULT_INCOMPATIBLE;
  }

  std::copy(res_vals.begin(), res_vals.end(), _out.begin());
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE RealtimeCommander::readANG(
  std::array<float, 3> & _out,
  const MODE mode) const noexcept
{
  if (mode == MODE::NORMAL) {
    static const char write_buf[] = "?ANG\r";
    this->packet_handler_->writePort(write_buf, sizeof(write_buf));
  }

  std::string response;
  if (!this->waitForResponse(PacketPool::PACKET_TYPE::ANG, response)) {
    return RESPONSE_STATE::ERROR_NO_RESPONSE;
  }

  std::vector<float> res_vals;
  if (!PacketPool::parseResponse(response, res_vals)) {
    return RESPONSE_STATE::ERROR_PARSE_FAILED;
  }

  if (res_vals.size() != _out.size()) {
    return RESPONSE_STATE::ERROR_PARSE_RESULT_INCOMPATIBLE;
  }

  std::copy(res_vals.begin(), res_vals.end(), _out.begin());
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE RealtimeCommander::startQuery(
  const uint32_t every_ms) const noexcept
{
  if (every_ms < 10) {
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }

  static char write_buf[128];
  int cx = snprintf(
    write_buf, sizeof(write_buf), "# %d\r", every_ms);
  this->packet_handler_->writePort(write_buf, cx);
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE RealtimeCommander::clearQuery() const noexcept
{
  static const char write_buf[] = "# C\r";
  this->packet_handler_->writePort(write_buf, sizeof(write_buf));
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE RealtimeCommander::writeANG(
  const int idx, const int val) const noexcept
{
  if (idx < 1 || idx > 3) {
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  static char write_buf[128];
  int cx = snprintf(
    write_buf, sizeof(write_buf), "!ANG %d %d\r", idx, val);
  this->packet_handler_->writePort(write_buf, cx);
  return RESPONSE_STATE::OK;
}

bool RealtimeCommander::waitForResponse(
  const PacketPool::PACKET_TYPE & type,
  std::string & response) const noexcept
{
  bool has_response = false;
  const auto start = this->clock_->now();
  while (this->clock_->now() - start < this->TIMEOUT_) {
    if (this->packet_handler_->getBytesAvailable() < 1) {
      rclcpp::sleep_for(10ms);
      continue;
    }
    this->packet_handler_->readPortIntoQueue();
    if (this->packet_handler_->takePacket(type, response)) {
      has_response = true;
      break;
    }
    rclcpp::sleep_for(10ms);
  }

  return has_response;
}

const rclcpp::Logger RealtimeCommander::getLogger() noexcept
{
  return this->logging_interface_->get_logger();
}

}  // namespace  mgs1600gy_interface
