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

#include "mgs1600gy_interface/port_handler_base.hpp"


namespace mgs1600gy_interface
{

class PortHandler final : public PortHandlerBase
{
private:
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

  size_t getBytesAvailable() const override;
  size_t readPort(char * const, const size_t) const override;
  size_t writePort(const char * const, const size_t) const override;

private:
  bool setupPort(const speed_t);
  speed_t getCFlagBaud(const int) const noexcept;

  static const std::string fixEscapeSequence(const std::string &);
  static const rclcpp::Logger getLogger();
};

}  // namespace mgs1600gy_interface
