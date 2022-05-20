"""Launch sensor."""
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch sensor."""
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value=TextSubstitution(
            text='/dev/serial/by-id/usb-Roboteq_Magnetic_Sensor_48F263793238-if00')
    )

    container = ComposableNodeContainer(
        name='image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mgs1600gy_node',
                plugin='mgs1600gy_node::ShowNode',
                name='mgs1600gy_viewer',
            ),
            ComposableNode(
                package='mgs1600gy_node',
                plugin='mgs1600gy_node::Mgs1600gyNode',
                name='mgs1600gy',
                parameters=[{'dev': LaunchConfiguration('dev')}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        dev_arg,
        container,
    ])
