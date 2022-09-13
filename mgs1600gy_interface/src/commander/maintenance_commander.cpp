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
  std::shared_ptr<PacketHandler> _packet_handler,
  std::chrono::nanoseconds timeout)
: packet_handler_(_packet_handler),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(rclcpp::Duration(timeout))
{
}

const rclcpp::Logger MaintenanceCommander::getLogger() noexcept
{
  return rclcpp::get_logger("MaintenanceCommander");
}
}  // namespace mgs1600gy_interface
