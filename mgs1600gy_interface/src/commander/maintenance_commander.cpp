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

#include "mgs1600gy_interface/commander/maintenance_commander.hpp"

namespace mgs1600gy_interface
{

MaintenanceCommander::MaintenanceCommander(
  PacketHandler::SharedPtr packet_handler,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
  const std::chrono::nanoseconds & timeout)
: packet_handler_(packet_handler),
  logging_interface_(logger),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(timeout)
{
}

RESPONSE_STATE MaintenanceCommander::writeZERO() const noexcept
{
  static const char write_buf[] = "%ZERO\r";
  this->packet_handler_->writePort(write_buf, sizeof(write_buf));
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE MaintenanceCommander::writeGZER() const noexcept
{
  static const char write_buf[] = "%GZER\r";
  this->packet_handler_->writePort(write_buf, sizeof(write_buf));
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE MaintenanceCommander::writeCLSAV() const noexcept
{
  static const char write_buf[] = "%CLSAV\r";
  this->packet_handler_->writePort(write_buf, sizeof(write_buf));
  return RESPONSE_STATE::OK;
}

const rclcpp::Logger MaintenanceCommander::getLogger() noexcept
{
  return this->logging_interface_->get_logger();
}
}  // namespace mgs1600gy_interface
