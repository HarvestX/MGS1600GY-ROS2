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
  const std::reference_wrapper<const sensor_msgs::msg::Imu::UniquePtr>
  imu_msg_ptr_ref
)
{
  std::cout << "IMU: " << std::setprecision(2);
  std::cout << " x: " << imu_msg_ptr_ref.get()->orientation.x;
  std::cout << " y: " << imu_msg_ptr_ref.get()->orientation.y;
  std::cout << " z: " << imu_msg_ptr_ref.get()->orientation.z;
  std::cout << " w: " << imu_msg_ptr_ref.get()->orientation.w;
  std::cout << std::endl;
}

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);

  const std::string port_name =
    "/dev/ttyACM0";

  auto mgs1600gy_interface = std::make_unique<
    mgs1600gy_interface::Mgs1600gyInterface>(
    port_name, 500ms);

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

  mgs1600gy_interface->startQueries(10);
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  std_msgs::msg::Header header;
  header.frame_id = "base_link";

  for (int i = 0; i < 10; ++i) {
    if (mgs1600gy_interface->read(PACKET_TYPE::ANG)) {
      header.stamp = rclcpp::Clock().now();
      mgs1600gy_interface->setOrientation(
        header, imu_msg);
      showImuData(imu_msg);
    }
    rclcpp::sleep_for(10ms);
  }

  mgs1600gy_interface->deactivate();

  return EXIT_SUCCESS;
}
