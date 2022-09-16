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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace mgs1600gy_node
{

class Imu2TfNode : public rclcpp::Node
{
private:
  std::string frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  explicit Imu2TfNode(const rclcpp::NodeOptions &);

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr);
};
}  // namespace mgs1600gy_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mgs1600gy_node::Imu2TfNode)
