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

#include "mgs1600gy_node/imu2tf_node.hpp"


namespace mgs1600gy_node
{
Imu2TfNode::Imu2TfNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu2tf_node", options)
{
  this->frame_id_ = this->declare_parameter("frame_id", "base_link");
  this->tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  this->imu_sub_ =
    this->create_subscription<Imu>(
    "imu", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&Imu2TfNode::onImu, this, std::placeholders::_1));
}

void Imu2TfNode::onImu(const Imu::ConstSharedPtr imu_msg)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = this->frame_id_;
  tf.header.stamp = imu_msg->header.stamp;
  tf.child_frame_id = imu_msg->header.frame_id;

  tf.transform.translation.set__x(0);
  tf.transform.translation.set__y(0);
  tf.transform.translation.set__z(0);

  tf.transform.rotation = imu_msg->orientation;

  this->tf_broadcaster_->sendTransform(tf);
}
}  // namespace mgs1600gy_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mgs1600gy_node::Imu2TfNode)
