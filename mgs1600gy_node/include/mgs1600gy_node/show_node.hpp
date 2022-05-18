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
#include <string>
#include <memory>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mgs1600gy_node
{

class ShowNode : public rclcpp::Node
{
private:
  const std::string WINDOW_NAME_ = "Sensor Data";
  image_transport::Subscriber image_sub_;

public:
  explicit ShowNode(const rclcpp::NodeOptions &);

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr);
};
}  // namespace mgs1600gy_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mgs1600gy_node::ShowNode)
