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


#include "mgs1600gy_interface/packet_handler.hpp"

namespace mgs1600gy_interface
{
PacketHandler::PacketHandler(PortHandlerBase const * const port_handler)
: port_handler_(port_handler),
  pool_(std::make_unique<PacketPool>())
{
}

ssize_t PacketHandler::writePort(char const * const buf, const size_t length) const
{
  return this->port_handler_->write(buf, length);
}

ssize_t PacketHandler::readPortIntoQueue()
{
  char buf[128];
  const ssize_t ret = this->port_handler_->read(buf, sizeof(buf));
  if (ret != -1) {
    this->pool_->enqueue(std::string(buf, ret));
  }
  return ret;
}

bool PacketHandler::takePacket(
  const PacketPool::PACKET_TYPE & packet_type, std::string & out)
{
  return this->pool_->takePacket(packet_type, out);
}


}  // namespace mgs1600gy_interface
