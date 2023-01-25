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

void showImuData(
  sensor_msgs::msg::Imu const * const imu_msg
)
{
  std::cout << std::setprecision(2);

  std::cout << "orientation:" << std::endl;
  std::cout << " x: " << imu_msg->orientation.x;
  std::cout << " y: " << imu_msg->orientation.y;
  std::cout << " z: " << imu_msg->orientation.z;
  std::cout << " w: " << imu_msg->orientation.w;
  std::cout << std::endl;

  std::cout << "velocity:" << std::endl;
  std::cout << " x: " << imu_msg->angular_velocity.x;
  std::cout << " y: " << imu_msg->angular_velocity.y;
  std::cout << " z: " << imu_msg->angular_velocity.z;
  std::cout << std::endl;
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
    return "ShowImu";
  }
};

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;  // NOLINT
  rclcpp::init(argc, argv);

  const std::string port_name = "/dev/ttyACM0";

  const auto logger = std::make_shared<Logger>();
  auto mgs1600gy_interface =
    std::make_unique<mgs1600gy_interface::Mgs1600gyInterface>(port_name, logger, 500ms);

  if (!mgs1600gy_interface->init()) {
    return EXIT_FAILURE;
  }
  if (!mgs1600gy_interface->activate()) {
    return EXIT_FAILURE;
  }

  using PACKET_TYPE = mgs1600gy_interface::PacketPool::PACKET_TYPE;

  if (!mgs1600gy_interface->setQueries(PACKET_TYPE::ANG)) {
    return EXIT_FAILURE;
  }

  if (!mgs1600gy_interface->setQueries(PACKET_TYPE::GY)) {
    return EXIT_FAILURE;
  }

  mgs1600gy_interface->startQueries(10);
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  std_msgs::msg::Header header;
  header.frame_id = "base_link";

  for (int i = 0; i < 100; ++i) {
    if (mgs1600gy_interface->read(PACKET_TYPE::ANG) && mgs1600gy_interface->read(PACKET_TYPE::GY)) {
      header.stamp = rclcpp::Clock().now();
      const auto imu_msg = mgs1600gy_interface->getImu(header);
      showImuData(imu_msg.get());
    }
    rclcpp::sleep_for(10ms);
  }

  mgs1600gy_interface->deactivate();

  return EXIT_SUCCESS;
}
