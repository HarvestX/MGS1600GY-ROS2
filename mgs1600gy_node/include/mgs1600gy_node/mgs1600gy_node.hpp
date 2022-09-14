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


#pragma once

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mgs1600gy_interface/mgs1600gy_interface.hpp>

namespace mgs1600gy_node
{

class Mgs1600gyNode : public rclcpp::Node
{
private:
  const float SENSOR_MIN_;
  const float SENSOR_MAX_;
  const bool FLIP_;

  std::string camera_name_;
  std::string camera_base_link_;
  std::string camera_magnet_link_;
  std::string camera_gyro_link_;

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mgs1600gy_interface::Mgs1600gyInterface> interface_;
  cv::Mat sensor_data_;

public:
  explicit Mgs1600gyNode(const rclcpp::NodeOptions &);

private:
  void onConnect();
};
}  // namespace mgs1600gy_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mgs1600gy_node::Mgs1600gyNode)
