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

from mgs1600gy_bringup.node_generator import mgs1600gy_node


def generate_launch_description():
    """Launch sensor."""
    launch_args = [
        DeclareLaunchArgument(
            'dev',
            default_value=TextSubstitution(
                text='/dev/serial/by-id/'
                'usb-Roboteq_Magnetic_Sensor_48F263793238-if00')),
        DeclareLaunchArgument(
            'sensor_min',
            default_value=TextSubstitution(text='-2000')),
        DeclareLaunchArgument(
            'sensor_max',
            default_value=TextSubstitution(text='2000')),
        DeclareLaunchArgument(
            'show',
            default_value=TextSubstitution(text='True'),
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value=TextSubstitution(text='mgs'),
        ),
        DeclareLaunchArgument(
            'flip',
            default_value=TextSubstitution(text='False'),
        )
    ]

    nodes = mgs1600gy_node.load(
        show=LaunchConfiguration('show'),
        dev=LaunchConfiguration('dev'),
        namespace=LaunchConfiguration('namespace'),
        sensor_min=LaunchConfiguration('sensor_min'),
        sensor_max=LaunchConfiguration('sensor_max'),
        flip=LaunchConfiguration('flip'),
    )

    return LaunchDescription(launch_args + nodes)
