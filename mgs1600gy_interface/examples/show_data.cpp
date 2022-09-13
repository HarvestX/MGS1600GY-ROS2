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

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);

  const std::string port_name =
    "/dev/ttyUSB0";

  auto mgs1600gy_interface = std::make_unique<
    mgs1600gy_interface::Mgs1600gyInterface>(
    port_name, 500ms);

  if (!mgs1600gy_interface->init()) {
    return EXIT_FAILURE;
  }
  if (!mgs1600gy_interface->activate()) {
    return EXIT_FAILURE;
  }

  if (!mgs1600gy_interface->setQueries(
      mgs1600gy_interface::PacketPool::PACKET_TYPE::MZ))
  {
    return EXIT_FAILURE;
  }

  if (!mgs1600gy_interface->setQueries(
      mgs1600gy_interface::PacketPool::PACKET_TYPE::ANG))
  {
    return EXIT_FAILURE;
  }

  mgs1600gy_interface->startQueries(10);

  for (int i = 0; i < 10; ++i) {
    if (!mgs1600gy_interface->readAll()) {
      // Do nothing
    }
    rclcpp::sleep_for(10ms);
  }

  mgs1600gy_interface->deactivate();

  return EXIT_SUCCESS;
}
