// Copyright 2023 HarvestX Inc.
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

#include <cv_bridge/cv_bridge.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mgs1600gy_interface/mgs1600gy_interface.hpp>

namespace mgs1600gy_control
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using rclcpp_lifecycle::State;
using Imu = sensor_msgs::msg::Imu;
using Image = sensor_msgs::msg::Image;
using PT = mgs1600gy_interface::PacketPool::PACKET_TYPE;

class Mgs1600gySensor : public hardware_interface::SensorInterface
{
private:
  using Interface = mgs1600gy_interface::Mgs1600gyInterface;

  double SENSOR_MIN_, SENSOR_MAX_;
  std::string NAME_, BASE_LINK_, MAGNET_LINK_;
  bool imu_included_;
  const uint32_t update_rate_ = 30;

  std::vector<double> imu_states_;

  double state_if_val_ = std::numeric_limits<double>::quiet_NaN();    // Not used
  std::array<uint8_t, 16 * 3> img_data_;

  std::string shm_key_;
  std::unique_ptr<boost::interprocess::shared_memory_object> shm_;
  std::unique_ptr<boost::interprocess::mapped_region> map_;

  Interface::UniquePtr interface_;
  cv::Mat sensor_data_;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Mgs1600gySensor)
  ~Mgs1600gySensor();

  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  CallbackReturn on_configure(const State &) override;
  CallbackReturn on_activate(const State &) override;
  CallbackReturn on_deactivate(const State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace mgs1600gy_control
