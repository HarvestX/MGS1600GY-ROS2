// Copyright 2022 HarvestX Inc
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

#include "mgs1600gy_tool/gyro_calibrator.hpp"


namespace mgs1600gy_tool
{
GyroCalibrator::GyroCalibrator(const rclcpp::NodeOptions & options)
: rclcpp::Node("gyro_calibrator", options)
{
  const std::string dev = this->declare_parameter(
    "dev", "/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00");

  using namespace std::chrono_literals;  // NOLINT
  this->interface_ = std::make_unique<mgs1600gy_interface::Mgs1600gyInterface>(
    dev, 500ms);

  if (!this->interface_->init()) {
    exit(EXIT_FAILURE);
    return;
  }
  if (!this->interface_->activate()) {
    exit(EXIT_FAILURE);
    return;
  }
}

GyroCalibrator::~GyroCalibrator()
{
  this->interface_->deactivate();
}

void GyroCalibrator::calibrate()
{
  RCLCPP_INFO(this->get_logger(), "Start gyro calibration...");
  if (!this->interface_->calibrateGyro()) {
    RCLCPP_ERROR(this->get_logger(), "Calibration failed.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Gyro sensor successfully calibrated!");

  if (!this->interface_->setAllAngleZero()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set ANG zero");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Calibration finished");
}
}  // namespace mgs1600gy_tool

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto calibrator = std::make_unique<mgs1600gy_tool::GyroCalibrator>(options);
  calibrator->calibrate();
}
