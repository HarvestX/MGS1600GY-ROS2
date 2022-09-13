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

#include "mgs1600gy_interface/mgs1600gy_interface.hpp"


namespace mgs1600gy_interface
{

Mgs1600gyInterface::Mgs1600gyInterface(
  const std::string & _port_name,
  const rclcpp::Duration & _timeout)
: TIMEOUT_(_timeout)
{
  this->port_handler_ =
    std::make_unique<PortHandler>(_port_name);
}

bool Mgs1600gyInterface::init()
{
  // Dry run to check can open port or not
  if (!this->port_handler_->openPort()) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Failed to open port");
    return false;
  }
  this->port_handler_->closePort();
  return true;
}

bool Mgs1600gyInterface::activate()
{
  if (!this->port_handler_->openPort()) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Failed to open port");
    return false;
  }

  this->packet_handler_ =
    std::make_shared<PacketHandler>(
    this->port_handler_.get());

  this->realtime_commander_ =
    std::make_unique<RealtimeCommander>(
    this->packet_handler_,
    this->TIMEOUT_);

  this->mode_ = RealtimeCommander::MODE::NORMAL;
  this->queries_.clear();
  this->realtime_commander_->clearQuery();

  return true;
}

bool Mgs1600gyInterface::deactivate()
{
  if (this->mode_ == RealtimeCommander::MODE::QUERY) {
    this->stopQueries();
  }

  this->port_handler_->closePort();
  this->packet_handler_ = nullptr;
  this->realtime_commander_ = nullptr;

  return true;
}

bool Mgs1600gyInterface::setQueries(
  const PacketPool::PACKET_TYPE & packet_type) noexcept
{
  if (!this->read(packet_type)) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Failed to set query: %s",
      PacketPool::packetTypeToString(packet_type).c_str());
    return false;
  }
  RCLCPP_INFO(
    this->getLogger(),
    "Packet type: %s successfully stocked.",
    PacketPool::packetTypeToString(packet_type).c_str());
  this->queries_.emplace_back(packet_type);
  return true;
}

bool Mgs1600gyInterface::startQueries(const uint32_t & every_ms) noexcept
{
  if (this->queries_.empty()) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Query is empty");
    return false;
  }

  std::stringstream query_ss;
  for (size_t i = 0; i < this->queries_.size(); ++i) {
    query_ss <<
      PacketPool::packetTypeToString(this->queries_.at(i)) <<
      ", ";
  }
  RCLCPP_INFO(
    this->getLogger(),
    "Following Commands will repeatedly executed [%s]",
    query_ss.str().c_str());

  const bool state = this->processResponse(
    this->realtime_commander_->startQuery(every_ms));

  if (state) {
    this->mode_ = RealtimeCommander::MODE::QUERY;
  }
  return state;
}

bool Mgs1600gyInterface::stopQueries() noexcept
{
  const bool ret =
    this->processResponse(this->realtime_commander_->clearQuery());
  if (ret) {
    this->mode_ = RealtimeCommander::MODE::NORMAL;
    this->queries_.clear();
  }
  return ret;
}

bool Mgs1600gyInterface::read(const PacketPool::PACKET_TYPE & packet_type)
{
  RESPONSE_STATE state;
  switch (packet_type) {
    case PacketPool::PACKET_TYPE::MZ:
      {
        state = this->realtime_commander_->readMZ(
          this->mz_data_, this->mode_);
        break;
      }
    case PacketPool::PACKET_TYPE::ANG:
      {
        state = this->realtime_commander_->readANG(
          this->ang_data_, this->mode_);
        break;
      }
    default:
      {
        RCLCPP_ERROR(
          this->getLogger(),
          "Invalid packet type given");
        return false;
      }
  }

  return this->processResponse(state);
}

bool Mgs1600gyInterface::readAll()
{
  for (size_t i = 0;
    i < static_cast<size_t>(PacketPool::PACKET_TYPE::END_PACKET_TYPE); ++i)
  {
    if (!this->read(static_cast<PacketPool::PACKET_TYPE>(i))) {
      return false;
    }
  }
  return true;
}


void Mgs1600gyInterface::getImage(
  cv::Mat * out,
  const float & MIN,
  const float & MAX,
  const bool & FLIP
) const noexcept
{
  Utils::convertBGR(
    this->mz_data_, out, MIN, MAX, FLIP);
}

const rclcpp::Logger Mgs1600gyInterface::getLogger() noexcept
{
  return rclcpp::get_logger("Mgs1600gyInterface");
}

bool Mgs1600gyInterface::processResponse(const RESPONSE_STATE & state) noexcept
{
  bool ret = false;
  switch (state) {
    case RESPONSE_STATE::OK:
      ret = true;
      break;
    case RESPONSE_STATE::ERROR_INVALID_INPUT:
      RCLCPP_ERROR(
        Mgs1600gyInterface::getLogger(),
        "Invalid input given.");
      break;
    case RESPONSE_STATE::ERROR_NO_RESPONSE:
      RCLCPP_ERROR(
        Mgs1600gyInterface::getLogger(),
        "Mgs1600gy is not responded.");
      break;
    case RESPONSE_STATE::ERROR_PARSE_FAILED:
      RCLCPP_ERROR(
        Mgs1600gyInterface::getLogger(),
        "Failed to parse received response.");
      break;
    case RESPONSE_STATE::ERROR_PARSE_RESULT_INCOMPATIBLE:
      RCLCPP_ERROR(
        Mgs1600gyInterface::getLogger(),
        "Responded parameters length is incompatible.");
      break;
    case RESPONSE_STATE::ERROR_UNKNOWN:
      RCLCPP_ERROR(
        Mgs1600gyInterface::getLogger(),
        "Unknown error.");
      break;
    default:
      RCLCPP_ERROR(
        Mgs1600gyInterface::getLogger(),
        "Invalid state given");
  }
  return ret;
}

}  // namespace mgs1600gy_interface
