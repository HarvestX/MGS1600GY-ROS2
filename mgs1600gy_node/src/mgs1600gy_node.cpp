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

#include "mgs1600gy_node/mgs1600gy_node.hpp"


namespace mgs1600gy_node
{
Mgs1600gyNode::Mgs1600gyNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("mgs1600gy_node", node_options),
  SENSOR_MIN_(this->declare_parameter("sensor_min", -2000)),
  SENSOR_MAX_(this->declare_parameter("sensor_max", 2000)),
  FLIP_(this->declare_parameter("flip", false)),
  NAME_(this->declare_parameter("name", "mgs1600gy")),
  BASE_LINK_(this->NAME_ + "_link"),
  MAGNET_LINK_(this->NAME_ + "_magnet_link")
{
  const float init_r = this->declare_parameter("roll", NAN);
  const float init_p = this->declare_parameter("pitch", NAN);
  const float init_y = this->declare_parameter("yaw", NAN);

  const auto & orient_cov = this->declare_parameter<std::vector<double>>(
    "orientation_covariance", {0.0, 0.0, 0.0});
  const auto & ang_vel_cov = this->declare_parameter<std::vector<double>>(
    "angular_velocity_covariance", {0.0, 0.0, 0.0});

  const std::string DEV = this->declare_parameter(
    "dev", "/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00");

  RCLCPP_INFO(this->get_logger(), "Base link name: %s", this->BASE_LINK_.c_str());
  RCLCPP_INFO(this->get_logger(), "Magnet link name: %s", this->MAGNET_LINK_.c_str());
  RCLCPP_INFO(this->get_logger(), "Selected device: %s", DEV.c_str());
  RCLCPP_INFO(this->get_logger(), "Read range: %.0f - %.0f", this->SENSOR_MIN_, this->SENSOR_MAX_);
  if (this->FLIP_) {
    RCLCPP_INFO(this->get_logger(), "Flip enabled");
  }

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
    RCLCPP_INFO_STREAM(this->get_logger(), log_ss.str());
  }

  // Setup Sensor to Read Magnetic data
  using namespace std::chrono_literals;  // NOLINT
  this->interface_ = std::make_unique<mgs1600gy_interface::Mgs1600gyInterface>(
    DEV, this->get_node_logging_interface(), 500ms);
  if (!this->interface_->init()) {
    exit(EXIT_FAILURE);
    return;
  }
  if (!this->interface_->activate()) {
    exit(EXIT_FAILURE);
    return;
  }

  // Initialize angles
  using AxisIndex = mgs1600gy_interface::Mgs1600gyInterface::AxisIndex;
  if (!std::isnan(init_r) && !this->interface_->setAngle(AxisIndex::ROLL, init_r)) {
    exit(EXIT_FAILURE);
  }
  if (!std::isnan(init_p) && !this->interface_->setAngle(AxisIndex::PITCH, init_p)) {
    exit(EXIT_FAILURE);
  }
  if (!std::isnan(init_y) && !this->interface_->setAngle(AxisIndex::YAW, init_y)) {
    exit(EXIT_FAILURE);
  }

  this->interface_->stopQueries();
  const auto queries = {PT::MZ, PT::ANG, PT::GY};
  for (const auto & query : queries) {
    if (!this->interface_->setQueries(query)) {
      exit(EXIT_FAILURE);
    }
  }

  const uint32_t interval = 10;
  this->interface_->startQueries(interval);
  this->sensor_data_ = cv::Mat(1, 16, CV_8UC3);

  this->image_pub_ = image_transport::create_publisher(
    this, "image", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  this->interface_->setImuCovariance(orient_cov, ang_vel_cov);
  this->imu_pub_ = create_publisher<Imu>("imu", rclcpp::SensorDataQoS());

  this->image_timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(interval),
    std::bind(&Mgs1600gyNode::onImageTimer, this));

  this->imu_timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(interval),
    std::bind(&Mgs1600gyNode::onImuTimer, this));
}


void Mgs1600gyNode::onImageTimer()
{
  if (!this->interface_->read(PT::MZ)) {
    return;
  }

  this->interface_->getImage(
    &this->sensor_data_, this->SENSOR_MIN_, this->SENSOR_MAX_, this->FLIP_);
  std_msgs::msg::Header header;
  header.frame_id = this->MAGNET_LINK_;
  header.stamp = this->get_clock()->now();
  Image::SharedPtr image_msg = cv_bridge::CvImage(header, "bgr8", this->sensor_data_).toImageMsg();
  this->image_pub_.publish(image_msg);
}

void Mgs1600gyNode::onImuTimer()
{
  if (!this->interface_->read(PT::ANG) || !this->interface_->read(PT::GY)) {
    return;
  }

  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  std_msgs::msg::Header header;
  header.frame_id = this->BASE_LINK_;
  header.stamp = this->get_clock()->now();

  this->imu_pub_->publish(this->interface_->getImu(header));
}

}  // namespace mgs1600gy_node
