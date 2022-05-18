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
#include <opencv2/opencv.hpp>
#include <mgs1600gy_interface/command_handler.hpp>
#include <mgs1600gy_interface/converter.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mgs1600gy_node
{

class Mgs1600gyNode : public rclcpp::Node
{
private:
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mgs1600gy_interface::CommandHandler> command_handler_;
  std::unique_ptr<mgs1600gy_interface::Converter<int,
    mgs1600gy_interface::MAGNET_SENSOR_NUM>> data_converter_;
  cv::Mat sensor_data_;

public:
  explicit Mgs1600gyNode(const rclcpp::NodeOptions &);

private:
  void onConnect();
};
}  // namespace mgs1600gy_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mgs1600gy_node::Mgs1600gyNode)
