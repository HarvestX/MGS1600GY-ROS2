"""Display robot joint states."""
# Copyright 2022 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from mgs1600gy_bringup.node_generator import robot_state_publisher as rsp
from mgs1600gy_bringup.node_generator import rviz2


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch rviz display."""
    robot_state_publisher_node: Node = rsp.load_node(
        filename='mgs1600gy.urdf.xacro')

    rviz_node: Node = rviz2.load_node('display.rviz')

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])
