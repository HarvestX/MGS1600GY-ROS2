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
: rclcpp::Node("mgs1600gy_node", node_options)
{
  const std::string DEV = this->declare_parameter(
    "dev", "/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00");
  const int SENSOR_MIN = this->declare_parameter("sensor_min", -2000);
  const int SENSOR_MAX = this->declare_parameter("sensor_max", 2000);
  const bool FLIP = this->declare_parameter("flip", false);

  RCLCPP_INFO(
    this->get_logger(),
    "Selected device: %s", DEV.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Read range: %d - %d", SENSOR_MIN, SENSOR_MAX);
  if (FLIP) {
    RCLCPP_INFO(
      this->get_logger(),
      "Flip enabled");
  }
  auto port_handler = std::make_unique<mgs1600gy_interface::PortHandler>(DEV);
  if (!port_handler->openPort()) {
    rclcpp::shutdown();
  }

  // Setup Sensor to Read Magnetic data
  this->command_handler_ =
    std::make_unique<mgs1600gy_interface::CommandHandler>(
    std::move(port_handler));
  command_handler_->readMZ();
  command_handler_->writeRepeatEvery(100);

  this->data_converter_ =
    std::make_unique<mgs1600gy_interface::Converter<
        int, mgs1600gy_interface::MAGNET_SENSOR_NUM>>(SENSOR_MIN, SENSOR_MAX, FLIP);
  this->sensor_data_ = this->data_converter_->yieldBaseRGBCvMat();

  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  this->image_pub_ = image_transport::create_publisher(
    this,
    "image",
    sensor_qos.get_rmw_qos_profile());

  using namespace std::chrono_literals;
  this->timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    100ms, std::bind(&Mgs1600gyNode::onConnect, this));
}


void Mgs1600gyNode::onConnect()
{
  std::string recv;
  if (!this->command_handler_->recv(recv)) {
    return;
  }
  if (!this->command_handler_->parseMgData(recv)) {
    return;
  }
  this->data_converter_->convertRGB(
    command_handler_->mg_data, this->sensor_data_);

  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::SharedPtr image_msg =
    cv_bridge::CvImage(header, "rgb8", this->sensor_data_).toImageMsg();
  this->image_pub_.publish(image_msg);
}

}  // namespace mgs1600gy_node
