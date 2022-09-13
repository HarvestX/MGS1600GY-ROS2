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

#include <opencv2/opencv.hpp>
#include "mgs1600gy_interface/converter.hpp"

int main()
{
  const rclcpp::Logger logger = rclcpp::get_logger("ShowImage");
  const std::string port_name =
    "/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00";
  auto port_handler =
    std::make_unique<mgs1600gy_interface::PortHandler>(port_name);

  if (!port_handler->openPort()) {
    RCLCPP_ERROR(logger, "Failed to open the port!");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(
    logger, "Succeeded to open the port!");
  RCLCPP_INFO(
    logger, "BaudRate: %d", port_handler->getBaudRate());

  auto command_handler =
    std::make_unique<mgs1600gy_interface::CommandHandler>(
    std::move(port_handler));
  auto converter =
    std::make_unique<mgs1600gy_interface::Converter<int, 16>>(0, 2000);
  auto cv_img = converter->yieldBaseBlurCvMat();

  command_handler->readMZ();

  command_handler->writeRepeatEvery(100);

  std::string receive_string;
  const std::string window_name = "Magnet Sensor";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(window_name, cv::Size(480, 10));
  while (1) {
    if (!command_handler->recv(receive_string)) {
      continue;
    }
    if (!command_handler->parseMgData(receive_string)) {
      continue;
    }
    converter->convertBlur(command_handler->mg_data, cv_img);
    cv::imshow(window_name, cv_img);
    cv::waitKey(1);
  }

  return EXIT_SUCCESS;
}
