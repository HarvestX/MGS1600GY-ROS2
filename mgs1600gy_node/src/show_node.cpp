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

#include "mgs1600gy_node/show_node.hpp"

namespace mgs1600gy_node
{

ShowNode::ShowNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("show_node", node_options)
{
  this->image_sub_ = image_transport::create_subscription(
    this, "/image", std::bind(&ShowNode::onImage, this, std::placeholders::_1),
    "raw", rmw_qos_profile_sensor_data);

  cv::namedWindow(this->WINDOW_NAME_, cv::WINDOW_NORMAL);
  cv::resizeWindow(this->WINDOW_NAME_, cv::Size(480, 10));
}

void ShowNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv::Mat cv_image;
  try {
    cv_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::imshow(this->WINDOW_NAME_, cv_image);
  cv::waitKey(1);
}
}  // namespace mgs1600gy_node
