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

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <string>
#include <rclcpp/rclcpp.hpp>


namespace mgs1600gy_interface
{

class PortHandler
{
private:
  const rclcpp::Logger logger_ = rclcpp::get_logger("PortHandler");

  int socket_fd_;
  int baudrate_;
  std::string port_name_;

public:
  PortHandler() = delete;
  explicit PortHandler(const std::string &);
  bool openPort();
  void closePort();
  bool setBaudRate(const int);
  int getBaudRate() const noexcept;
  std::string getPortName() const noexcept;

  int getBytesAvailable();
  int readPort(uint8_t * const, int);
  int readPort(char * const, int);
  int writePort(const uint8_t * const, int);
  int writePort(const char * const, int);

private:
  bool setupPort(const speed_t);
  speed_t getCFlagBaud(const int) const noexcept;
};

}  // namespace mgs1600gy_interface
