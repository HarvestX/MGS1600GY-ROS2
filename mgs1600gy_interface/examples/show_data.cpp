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


#include "mgs1600gy_interface/port_handler.hpp"
#include "mgs1600gy_interface/command_handler.hpp"

int main()
{
  std::cout << "Hello World!" << std::endl;
  const rclcpp::Logger logger = rclcpp::get_logger("main");
  const std::string port_name =
    "/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00";

  auto port_handler =
    std::make_unique<mgs1600gy_interface::PortHandler>(port_name);

  bool res = port_handler->openPort();
  if (!res) {
    RCLCPP_ERROR(
      logger, "Failed to open the port!");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(
    logger, "Succeeded to open the port!");
  RCLCPP_INFO(
    logger, "BaudRate: %d",
    port_handler->getBaudRate());

  auto command_handler =
    std::make_unique<mgs1600gy_interface::CommandHandler>(
    std::move(port_handler));

  command_handler->readMZ();
  command_handler->readANG();

  command_handler->writeRepeatEvery(100);

  std::string receive_string;
  for (int i = 0; i < 10; ++i) {
    // Handle gyro data
    command_handler->recv(receive_string);
    if (command_handler->parseGyroData(receive_string)) {
      command_handler->showGyroData();
    }

    // Handle magnet data
    command_handler->recv(receive_string);
    if (command_handler->parseMgData(receive_string)) {
      command_handler->showMgData();
    }
  }

  return EXIT_SUCCESS;
}
