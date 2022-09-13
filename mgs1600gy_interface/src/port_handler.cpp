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


namespace mgs1600gy_interface
{
PortHandler::PortHandler(const std::string & port_name)
: socket_fd_(-1),
  baudrate_(115200),
  port_name_(port_name)
{
}

bool PortHandler::openPort()
{
  return this->setBaudRate(this->baudrate_);
}

void PortHandler::closePort()
{
  if (this->socket_fd_ != -1) {
    close(socket_fd_);
  }
  socket_fd_ = -1;
}

bool PortHandler::setBaudRate(const int baudrate)
{
  const speed_t cflag_baud = this->getCFlagBaud(baudrate);

  this->closePort();

  if (cflag_baud <= 0) {
    // Invalid baudrate given
    return false;
  }
  this->baudrate_ = baudrate;
  return setupPort(cflag_baud);
}

int PortHandler::getBaudRate() const noexcept
{
  return this->baudrate_;
}

std::string PortHandler::getPortName() const noexcept
{
  return this->port_name_;
}

bool PortHandler::setupPort(const speed_t cflag_baud)
{
  struct termios new_tio;
  this->socket_fd_ =
    open(this->port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (socket_fd_ < 0) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Error opening serial port");
    return false;
  }

  // Clear struct for new port settings
  bzero(&new_tio, sizeof(new_tio));

  new_tio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  new_tio.c_iflag = IGNPAR;
  new_tio.c_oflag = 0;
  new_tio.c_lflag = 0;
  new_tio.c_cc[VTIME] = 0;
  new_tio.c_cc[VMIN] = 0;

  // Clean buffer and activate settings
  tcflush(this->socket_fd_, TCIFLUSH);
  tcsetattr(this->socket_fd_, TCSANOW, &new_tio);
  return true;
}

size_t PortHandler::getBytesAvailable() const
{
  int bytes_available;
  ioctl(this->socket_fd_, FIONREAD, &bytes_available);
  RCLCPP_DEBUG(
    this->getLogger(),
    "Available: %d", bytes_available);
  return static_cast<size_t>(bytes_available);
}

size_t PortHandler::readPort(char * packet, const size_t length) const
{
  const size_t ret = read(this->socket_fd_, packet, length);
  if (ret > 0) {
    RCLCPP_DEBUG(
      this->getLogger(),
      "Recv: %s Length: %zu",
      this->fixEscapeSequence(packet).c_str(), ret);
  }
  return ret;
}

size_t PortHandler::writePort(const char * packet, const size_t length) const
{
  RCLCPP_DEBUG(
    this->getLogger(),
    "Send: %s Length: %zu",
    this->fixEscapeSequence(packet).c_str(), length);
  return write(this->socket_fd_, packet, length);
}

speed_t PortHandler::getCFlagBaud(const int baudrate) const noexcept
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      RCLCPP_ERROR(
        this->getLogger(),
        "Invalid baudrate given: %d", baudrate);
      return -1;
  }
}

const std::string PortHandler::fixEscapeSequence(const std::string & in)
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

const rclcpp::Logger PortHandler::getLogger()
{
  return rclcpp::get_logger("PortHandler");
}

}  // namespace mgs1600gy_interface
