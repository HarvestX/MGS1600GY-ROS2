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

#include "mgs1600gy_interface/command_handler.hpp"

namespace mgs1600gy_interface
{
CommandHandler::CommandHandler(
  std::unique_ptr<PortHandler> port_handler
)
: rx_config_generator(std::make_unique<RxConfiguration>()),
  rx_realtime_generator(std::make_unique<RxRealtime>()),
  tx_buffer_generator(std::make_unique<TxBuffer>()),
  tx_config_generator(std::make_unique<TxConfiguration>()),
  tx_mainte_generator(std::make_unique<TxMaintenance>()),
  tx_realtime_generator(std::make_unique<TxRealtime>()),
  port_handler_(std::move(port_handler)),
  logger_(rclcpp::get_logger("CommandHandler")),
  mg_parser_(std::make_unique<Parser<int, MAGNET_SENSOR_NUM>>("MgParser")),
  gyro_parser_(std::make_unique<Parser<int, GYRO_SENSOR_NUM>>("GyroParser"))
{
  this->writeClear();
}

CommandHandler::~CommandHandler()
{
  this->writeClear();
}

bool CommandHandler::readMZ()
{
  std::string buf;
  if (!this->sendRecv(this->rx_realtime_generator->yieldMZ(), buf)) {
    return false;
  }
  if (!this->mg_parser_->parse(buf, this->mg_data)) {
    return false;
  }
  this->mg_parser_->showParsedResult(this->mg_data);
  return true;
}

bool CommandHandler::readANG()
{
  std::string buf;
  if (!this->sendRecv(this->rx_realtime_generator->yieldANG(), buf)) {
    return false;
  }
  if (!this->gyro_parser_->parse(buf, this->gyro_data)) {
    return false;
  }
  this->gyro_parser_->showParsedResult(this->gyro_data);
  return true;
}

bool CommandHandler::writeRepeat()
{
  return this->send(this->tx_buffer_generator->yieldRepeat());
}

bool CommandHandler::writeRepeatEvery(const int ms)
{
  return this->send(this->tx_buffer_generator->yieldRepeatEvery(ms));
}

bool CommandHandler::writeClear()
{
  return this->send(this->tx_buffer_generator->yieldClear());
}

bool CommandHandler::send(const std::string && command)
{
  this->port_handler_->writePort(command.data(), command.size());

  // TODO: Implement timeout here
  while (1) {
    if (this->port_handler_->getBytesAvailable() > 0) {
      break;
    }
  }

  // Take response
  std::string buf;
  buf.resize(command.size());
  this->port_handler_->readPort(buf.data(), buf.size());

  if (command != buf) {
    RCLCPP_ERROR(
      this->logger_,
      "Failed to send: %s",
      command.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->logger_,
    "Succeeded to send: %s",
    command.c_str());
  return true;
}

bool CommandHandler::sendRecv(const std::string && command, std::string & response)
{
  this->port_handler_->writePort(command.data(), command.size());

  // TODO: Implement timeout here
  while (1) {
    if (this->port_handler_->getBytesAvailable() > 0) {
      break;
    }
  }

  response.resize(100);
  this->port_handler_->readPort(response.data(), response.size());

  const std::string delimiter = "\r";

  // Check validity of the response, should have 2 '\r'
  if (std::count(response.begin(), response.end(), delimiter.c_str()[0]) != 2) {
    RCLCPP_ERROR(this->logger_, "Invalid Response");
    return false;
  }

  size_t command_pos = response.find(delimiter) + delimiter.length();
  const std::string ret_command =
    response.substr(0, command_pos);
  response.erase(0, command_pos);

  if (command != ret_command) {
    RCLCPP_ERROR(
      this->logger_, "Failed to send: %s", command.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->logger_, "Succeeded to send: %s", command.c_str());
  RCLCPP_INFO(
    this->logger_, "Receive: %s", response.c_str());
  return true;
}

bool CommandHandler::recv(std::string & response)
{
  // TODO: Implement timeout here
  while (1) {
    if (this->port_handler_->getBytesAvailable() > 0) {
      break;
    }
  }
  std::string buf;
  buf.resize(100);

  this->port_handler_->readPort(buf.data(), buf.size());

  const std::string delimiter = "\r";

  size_t pos = 0;
  std::string token;

  int num = 0;
  while ((pos = buf.find(delimiter)) != std::string::npos) {
    token = buf.substr(0, pos);
    if (!token.empty()) {
      break;
    }
    buf.erase(0, pos + delimiter.length());
    num++;
  }

  if (!token.empty()) {
    response = token;
    RCLCPP_INFO(
      this->logger_, "Receive: %s",
      response.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->logger_, "Failed to receive command");
  return false;
}

bool CommandHandler::parseMgData(const std::string & command)
{
  return this->mg_parser_->parse(command, this->mg_data);
}

bool CommandHandler::parseGyroData(const std::string & command)
{
  return this->gyro_parser_->parse(command, this->gyro_data);
}

void CommandHandler::showMgData()
{
  this->mg_parser_->showParsedResult(this->mg_data);
}

void CommandHandler::showGyroData()
{
  this->gyro_parser_->showParsedResult(this->gyro_data);
}

}  // namespace mgs1600gy_interface
