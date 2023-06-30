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
  using namespace std::chrono_literals;  // NOLINT
  using namespace mgs1600gy_interface;  // NOLINT
  rclcpp::init(argc, argv);

  const std::string port_name = "/dev/ttyUSB0";

  auto mgs1600gy_interface = std::make_unique<Mgs1600gyInterface>(port_name, 500ms);

  if (!mgs1600gy_interface->init()) {
    return EXIT_FAILURE;
  }
  if (!mgs1600gy_interface->activate()) {
    return EXIT_FAILURE;
  }

  if (!mgs1600gy_interface->setQueries(PacketPool::PACKET_TYPE::MZ)) {
    return EXIT_FAILURE;
  }

  mgs1600gy_interface->startQueries(10);

  cv::namedWindow("MGS1600gy", cv::WINDOW_NORMAL);
  cv::resizeWindow("MGS1600gy", cv::Size(640, 120));
  cv::Mat img(1, 16, CV_8UC3);
  for (int i = 0; i < 1000; ++i) {
    if (mgs1600gy_interface->read(PacketPool::PACKET_TYPE::MZ)) {
      mgs1600gy_interface->getImage(&img);
      cv::imshow("MGS1600gy", img);
      cv::waitKey(1);
    }
    rclcpp::sleep_for(10ms);
  }
  return EXIT_SUCCESS;
}
