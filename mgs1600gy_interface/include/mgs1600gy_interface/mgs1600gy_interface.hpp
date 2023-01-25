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

#include <tf2/convert.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <h6x_serial_interface/port_handler.hpp>

#include "mgs1600gy_interface/packet_handler.hpp"
#include "mgs1600gy_interface/utils.hpp"
// TODO(m12watanabe1a): Uncomment those after implementation
// #include "mgs1600gy_interface/commander/configuration_commander.hpp"
#include "mgs1600gy_interface/commander/maintenance_commander.hpp"
#include "mgs1600gy_interface/commander/realtime_commander.hpp"


namespace mgs1600gy_interface
{
using namespace std::chrono_literals;  // NOLINT
using Imu = sensor_msgs::msg::Imu;

class Mgs1600gyInterface
{
public:
  using UniquePtr = std::unique_ptr<Mgs1600gyInterface>;

  enum class AxisIndex
  {
    PITCH = 0,
    YAW = 1,
    ROLL = 2,
  };

private:
  using PortHandler = h6x_serial_interface::PortHandler;
  RealtimeCommander::MODE mode_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  const std::chrono::nanoseconds TIMEOUT_;

  PortHandler::UniquePtr port_handler_;
  PacketHandler::SharedPtr packet_handler_;
  RealtimeCommander::UniquePtr realtime_commander_;
  MaintenanceCommander::UniquePtr maintenance_commander_;

  std::vector<PacketPool::PACKET_TYPE> queries_;

  std::array<float, 16> mz_data_;
  cv::Mat img_data_;

  std::array<float, 3> ang_data_;
  std::array<float, 3> gy_data_;

public:
  Mgs1600gyInterface() = delete;
  explicit Mgs1600gyInterface(
    const std::string &,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr,
    const std::chrono::nanoseconds & = 5s);

  bool init();
  bool activate();
  bool deactivate();

  bool read(const PacketPool::PACKET_TYPE &);
  bool readAll();

  void getMzData(std::array<float, 16> &) const noexcept;
  void getAngData(std::array<float, 3> &) const noexcept;
  void getGyData(std::array<float, 3> &) const noexcept;

  Imu::UniquePtr getImu(const std_msgs::msg::Header &) const noexcept;
  void getImage(
    cv::Mat *, const float & = -2000, const float & = 2000,
    const bool & = false) const noexcept;

  // Query commands
  bool setQueries(const PacketPool::PACKET_TYPE &) noexcept;
  bool startQueries(const uint32_t &) noexcept;
  bool stopQueries() noexcept;
  // End Query Commands

  // Realtime Commands
  bool setAllAngleZero() const noexcept;
  bool setAngle(const AxisIndex &, const float &) const noexcept;
  // End Realtime Commands

  // Calibration Commands
  bool calibrateMagnet() const noexcept;
  bool calibrateGyro() const noexcept;
  // End Calibration Commands

private:
  const rclcpp::Logger getLogger() const noexcept;
  bool processResponse(const RESPONSE_STATE &) const noexcept;
};
}  // namespace mgs1600gy_interface
