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

#include "mgs1600gy_control/mgs1600gy_sensor.hpp"

namespace mgs1600gy_control
{
Mgs1600gySensor::~Mgs1600gySensor()
{
  boost::interprocess::shared_memory_object::remove(this->shm_->get_name());
  this->shm_.reset();
}

CallbackReturn Mgs1600gySensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  this->sensor_min_ = std::stof(info.hardware_parameters.at("sensor_min"));
  this->sensor_max_ = std::stof(info.hardware_parameters.at("sensor_max"));

  const std::string dev = info.hardware_parameters.at("dev");

  RCLCPP_INFO(this->getLogger(), "Selected device: %s", dev.c_str());
  RCLCPP_INFO(this->getLogger(), "Read range: %.0f - %.0f", this->sensor_min_, this->sensor_max_);

  // Setup Sensor to Read Magnetic data
  using namespace std::chrono_literals;  // NOLINT
  this->interface_ = std::make_unique<mgs1600gy_interface::Mgs1600gyInterface>(dev, 500ms);
  if (!this->interface_->init()) {
    RCLCPP_ERROR(
      this->getLogger(), "can't initialize interface");
    return CallbackReturn::ERROR;
  }
  if (!this->interface_->activate()) {
    RCLCPP_ERROR(
      this->getLogger(), "can't activate interface");
    return CallbackReturn::ERROR;
  }

  // Check if imu is included or not
  this->imu_included_ = false;
  for (const auto & sensor : info.sensors) {
    for (const auto & state_interface : sensor.state_interfaces) {
      if (state_interface.name == "orientation.x") {
        this->imu_frame_id_ = sensor.name;
        this->imu_included_ = true;
        break;
      }
    }
  }

  // BEGIN: image sensor
  for (const auto & sensor : info.sensors) {
    for (const auto & state_interface : sensor.state_interfaces) {
      if (state_interface.name == "image") {
        this->image_frame_id = sensor.name;
        this->shm_key_ = sensor.name + "-" + state_interface.name;
        break;
      }
    }
  }

  if (this->shm_key_.empty()) {
    RCLCPP_ERROR(
      this->getLogger(), "sensor with 'image' state interface does not exist");
    return CallbackReturn::ERROR;
  }

  this->shm_ = std::make_unique<boost::interprocess::shared_memory_object>(
    boost::interprocess::open_or_create, this->shm_key_.c_str(), boost::interprocess::read_write);
  this->shm_->truncate(this->img_data_.size());
  // END: image sensor

  // BEGIN: Set up interface
  this->interface_->stopQueries();
  std::vector<PT> queries;
  queries.push_back(PT::MZ);
  uint32_t interval = this->update_rate_;

  if (this->imu_included_) {
    // BEGIN: IMU sensor
    this->imu_states_.resize(
      this->info_.sensors.at(1).state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
    // END: IMU sensor

    // Add queries
    queries.push_back(PT::ANG);
    queries.push_back(PT::GY);

    // Set interval
    interval /= 3;
  }

  for (size_t i = 0; i < queries.size(); i++) {
    if (!this->interface_->setQueries(queries.at(i))) {
      return CallbackReturn::ERROR;
    }
  }

  this->interface_->startQueries(interval);
  // END: Set up interface

  return CallbackReturn::SUCCESS;
}

CallbackReturn Mgs1600gySensor::on_configure(const State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn Mgs1600gySensor::on_activate(const State &)
{
  this->map_ = std::make_unique<boost::interprocess::mapped_region>(
    *this->shm_, boost::interprocess::read_write);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Mgs1600gySensor::on_deactivate(const State &)
{
  this->map_.reset();

  if (!this->interface_->deactivate()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Mgs1600gySensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interface;

  // Image
  state_interface.emplace_back(
    hardware_interface::StateInterface(
      this->info_.sensors.at(0).name, "image", &this->state_if_val_));

  // IMU
  if (this->imu_included_) {
    std::array<std::string, 4> coordinate = {"x", "y", "z", "w"};

    for (size_t i = 0; i < 4; i++) {
      state_interface.emplace_back(
        hardware_interface::StateInterface(
          this->info_.sensors.at(1).name, "orientation." + coordinate.at(i % 4),
          &this->imu_states_.at(i)));
    }
    for (size_t i = 4; i < 7; i++) {
      state_interface.emplace_back(
        hardware_interface::StateInterface(
          this->info_.sensors.at(1).name, "angular_velocity." + coordinate.at(i % 4),
          &this->imu_states_.at(i)));
    }
    for (size_t i = 7; i < 10; i++) {
      state_interface.emplace_back(
        hardware_interface::StateInterface(
          this->info_.sensors.at(1).name, "linear_acceleration." + coordinate.at((i + 1) % 4),
          &this->imu_states_.at(i)));
    }
  }

  return state_interface;
}

return_type Mgs1600gySensor::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  // BEGIN: image sensor
  if (!this->interface_->read(PT::MZ)) {
    RCLCPP_ERROR(
      this->getLogger(), "can't read image data");
    return return_type::ERROR;
  }

  this->interface_->getImage(this->img_data_, this->sensor_min_, this->sensor_max_);

  if (this->map_) {
    this->state_if_val_ = 0.0;
    std::memcpy(
      this->map_->get_address(), this->img_data_.data(), this->map_->get_size());
  }
  // END: image sensor

  // BEGIN: IMU sensor
  if (this->imu_included_) {
    if (!this->interface_->read(PT::ANG) || !this->interface_->read(PT::GY)) {
      RCLCPP_ERROR(
        this->getLogger(), "can't read imu data");
      return return_type::ERROR;
    }

    std_msgs::msg::Header imu_header;
    imu_header.frame_id = this->imu_frame_id_;
    imu_header.stamp = time;
    Imu::UniquePtr imu_msg = this->interface_->getImu(imu_header);

    this->imu_states_.at(0) = imu_msg->orientation.x;
    this->imu_states_.at(1) = imu_msg->orientation.y;
    this->imu_states_.at(2) = imu_msg->orientation.z;
    this->imu_states_.at(3) = imu_msg->orientation.w;
    this->imu_states_.at(4) = imu_msg->angular_velocity.x;
    this->imu_states_.at(5) = imu_msg->angular_velocity.y;
    this->imu_states_.at(6) = imu_msg->angular_velocity.z;
    this->imu_states_.at(7) = imu_msg->linear_acceleration.x;
    this->imu_states_.at(8) = imu_msg->linear_acceleration.y;
    this->imu_states_.at(9) = imu_msg->linear_acceleration.z;
  }
  // END: IMU sensor

  return return_type::OK;
}

const rclcpp::Logger Mgs1600gySensor::getLogger() noexcept
{
  return rclcpp::get_logger("Mgs1600gySensor");
}
}  // namespace mgs1600gy_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mgs1600gy_control::Mgs1600gySensor, hardware_interface::SensorInterface)
