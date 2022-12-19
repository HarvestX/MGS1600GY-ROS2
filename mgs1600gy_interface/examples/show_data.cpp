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

void showMzData(const std::array<float, 16> & data)
{
  std::cout << "Mz: [";
  std::cout << std::setprecision(3);
  for (const auto val : data) {
    std::cout << val << " ";
  }
  std::cout << "]" << std::endl;
}

void showAngData(const std::array<float, 3> & data)
{
  std::cout << "Ang: [";
  std::cout << std::setprecision(5);
  for (const auto val : data) {
    std::cout << val << " ";
  }
  std::cout << "]" << std::endl;
}

class Logger : public rclcpp::node_interfaces::NodeLoggingInterface
{
public:
  rclcpp::Logger get_logger() const override
  {
    return rclcpp::get_logger(this->get_logger_name());
  }

  const char * get_logger_name() const override
  {
    return "ShowData";
  }
};

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;  // NOLINT
  rclcpp::init(argc, argv);

  const std::string port_name = "/dev/ttyUSB0";

  const auto logger = std::make_shared<Logger>();
  auto mgs1600gy_interface = std::make_unique<
    mgs1600gy_interface::Mgs1600gyInterface>(port_name, logger, 500ms);

  if (!mgs1600gy_interface->init()) {
    return EXIT_FAILURE;
  }
  if (!mgs1600gy_interface->activate()) {
    return EXIT_FAILURE;
  }

  using PACKET_TYPE = mgs1600gy_interface::PacketPool::PACKET_TYPE;

  if (!mgs1600gy_interface->setQueries(PACKET_TYPE::MZ)) {
    return EXIT_FAILURE;
  }

  if (!mgs1600gy_interface->setQueries(PACKET_TYPE::ANG)) {
    return EXIT_FAILURE;
  }

  mgs1600gy_interface->startQueries(10);
  std::array<float, 16> mz_data;
  std::array<float, 3> ang_data;

  for (int i = 0; i < 10; ++i) {
    if (mgs1600gy_interface->read(PACKET_TYPE::MZ)) {
      mgs1600gy_interface->getMzData(mz_data);
      showMzData(mz_data);
    }
    if (mgs1600gy_interface->read(PACKET_TYPE::ANG)) {
      mgs1600gy_interface->getAngData(ang_data);
      showAngData(ang_data);
    }
    rclcpp::sleep_for(10ms);
  }

  mgs1600gy_interface->deactivate();

  return EXIT_SUCCESS;
}
