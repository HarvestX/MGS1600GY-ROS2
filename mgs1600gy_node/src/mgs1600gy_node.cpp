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
  FLIP_(this->declare_parameter("flip", false))
{
  this->camera_name_ = this->declare_parameter("CAMERA_NAME", "mgs1600gy");
  this->camera_base_link_ = this->camera_name_ + "_link";
  this->camera_magnet_link_ = this->camera_name_ + "_magnet_link";
  this->camera_gyro_link_ = this->camera_name_ + "_gyro_link";

  const std::string DEV = this->declare_parameter(
    "dev", "/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00");


  RCLCPP_INFO(
    this->get_logger(),
    "Camera base link name: %s", this->camera_base_link_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Camera magnet link name: %s", this->camera_magnet_link_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Camera gyro link name: %s", this->camera_gyro_link_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Selected device: %s", DEV.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Read range: %.0f - %.0f", this->SENSOR_MIN_, this->SENSOR_MAX_);
  if (this->FLIP_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Flip enabled");
  }
  // Setup Sensor to Read Magnetic data
  using namespace std::chrono_literals;
  this->interface_ =
    std::make_unique<mgs1600gy_interface::Mgs1600gyInterface>(DEV, 500ms);
  if (!this->interface_->init()) {
    rclcpp::shutdown();
  }
  if (!this->interface_->activate()) {
    rclcpp::shutdown();
  }
  if (!this->interface_->setQueries(
      mgs1600gy_interface::PacketPool::PACKET_TYPE::MZ))
  {
    rclcpp::shutdown();
  }
  if (!this->interface_->startQueries(100)) {
    rclcpp::shutdown();
  }


  this->sensor_data_ = cv::Mat(1, 16, CV_8UC3);


  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  this->image_pub_ = image_transport::create_publisher(
    this,
    "image",
    sensor_qos.get_rmw_qos_profile());

  this->imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "imu",
    10);

  this->timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    100ms, std::bind(&Mgs1600gyNode::onConnect, this));
}


void Mgs1600gyNode::onConnect()
{
  std::string recv;
  if (!this->interface_->read(
      mgs1600gy_interface::PacketPool::PACKET_TYPE::MZ))
  {
    return;
  }

  this->interface_->getImage(
    &this->sensor_data_,
    this->SENSOR_MIN_, this->SENSOR_MAX_, this->FLIP_);
  std_msgs::msg::Header header;
  header.frame_id = this->camera_magnet_link_;
  header.stamp = this->get_clock()->now();
  sensor_msgs::msg::Image::SharedPtr image_msg =
    cv_bridge::CvImage(header, "bgr8", this->sensor_data_).toImageMsg();
  this->image_pub_.publish(image_msg);

  std::array<float, 3> ang_data;
  this->interface_->getRotation(ang_data);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = header;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(ang_data[0], ang_data[1], ang_data[2]);
  imu_msg.orientation = tf2::toMsg(myQuaternion);
  this->imu_pub_->publish(imu_msg);
}

}  // namespace mgs1600gy_node
