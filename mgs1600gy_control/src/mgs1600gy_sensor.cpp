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

  SENSOR_MIN_ = std::stof(info.hardware_parameters.at("sensor_min"));
  SENSOR_MAX_ = std::stof(info.hardware_parameters.at("sensor_max"));
  NAME_ = info.hardware_parameters.at("name");
  BASE_LINK_ = NAME_ + "_link";
  MAGNET_LINK_ = NAME_ + "_magnet_link";

  const float init_r = std::stof(info.hardware_parameters.at("roll"));
  const float init_p = std::stof(info.hardware_parameters.at("pitch"));
  const float init_y = std::stof(info.hardware_parameters.at("yaw"));

  const std::string DEV = info.hardware_parameters.at("dev");

  RCLCPP_INFO(this->getLogger(), "Base link name: %s", this->BASE_LINK_.c_str());
  RCLCPP_INFO(this->getLogger(), "Magnet link name: %s", this->MAGNET_LINK_.c_str());
  RCLCPP_INFO(this->getLogger(), "Selected device: %s", DEV.c_str());
  RCLCPP_INFO(this->getLogger(), "Read range: %.0f - %.0f", this->SENSOR_MIN_, this->SENSOR_MAX_);

  std::stringstream log_ss;
  if (!std::isnan(init_p)) {
    log_ss << "pitch: " << init_p << " ";
  }
  if (!std::isnan(init_r)) {
    log_ss << "roll: " << init_r << " ";
  }
  if (!std::isnan(init_y)) {
    log_ss << "yaw: " << init_y << " ";
  }
  if (!log_ss.str().empty()) {
    RCLCPP_INFO_STREAM(this->getLogger(), log_ss.str());
  }

  // BEGIN: image sensor
  for (const auto & sensor : info.sensors) {
    for (const auto & state_interface : sensor.state_interfaces) {
      if (state_interface.name == "image") {
        this->shm_key_ = sensor.name + "-" + state_interface.name;
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

  // BEGIN: IMU sensor
  this->imu_states_.resize(
    this->info_.sensors.at(1).state_interfaces.size(),
    std::numeric_limits<double>::quiet_NaN());
  // END: IMU sensor

  // Setup Sensor to Read Magnetic data
  using namespace std::chrono_literals;  // NOLINT
  this->interface_ = std::make_unique<mgs1600gy_interface::Mgs1600gyInterface>(DEV, 500ms);
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

  // Initialize angles
  using AxisIndex = mgs1600gy_interface::Mgs1600gyInterface::AxisIndex;
  if (!std::isnan(init_r) && !this->interface_->setAngle(AxisIndex::ROLL, init_r)) {
    // return CallbackReturn::ERROR;
  }
  if (!std::isnan(init_p) && !this->interface_->setAngle(AxisIndex::PITCH, init_p)) {
    // return CallbackReturn::ERROR;
  }
  if (!std::isnan(init_y) && !this->interface_->setAngle(AxisIndex::YAW, init_y)) {
    // return CallbackReturn::ERROR;
  }

  this->interface_->stopQueries();
  const auto queries = {PT::MZ, PT::ANG, PT::GY};
  for (const auto & query : queries) {
    if (!this->interface_->setQueries(query)) {
      return CallbackReturn::ERROR;
    }
  }

  const uint32_t interval = 10;
  this->interface_->startQueries(interval);
  this->sensor_data_ = cv::Mat(1, 16, CV_8UC3);

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
  std::array<std::string, 4> coordinate = {"x", "y", "z", "w"};

  state_interface.emplace_back(
    hardware_interface::StateInterface(
      this->info_.sensors.at(0).name, "image", &this->state_if_val_));

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

  return state_interface;
}

return_type Mgs1600gySensor::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!this->interface_->read(PT::MZ) || !this->interface_->read(PT::ANG) ||
    !this->interface_->read(PT::GY))
  {
    RCLCPP_ERROR(
      this->getLogger(), "can't read data");
    return return_type::ERROR;
  }
  // BEGIN: image sensor
  this->interface_->getImage(&this->sensor_data_, this->SENSOR_MIN_, this->SENSOR_MAX_);
  std_msgs::msg::Header image_header;
  image_header.frame_id = this->MAGNET_LINK_;
  image_header.stamp = time;
  Image::SharedPtr image_msg =
    cv_bridge::CvImage(image_header, "bgr8", this->sensor_data_).toImageMsg();

  for (size_t i = 0; i < image_msg->data.size(); i++) {
    this->img_data_.at(i) = image_msg->data.at(i);
  }

  if (this->map_) {
    this->state_if_val_ = 0.0;
    std::memcpy(
      this->map_->get_address(), this->img_data_.data(), this->map_->get_size());
  }
  // END: image sensor

  // BEGIN: IMU sensor
  std_msgs::msg::Header imu_header;
  imu_header.frame_id = this->BASE_LINK_;
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
