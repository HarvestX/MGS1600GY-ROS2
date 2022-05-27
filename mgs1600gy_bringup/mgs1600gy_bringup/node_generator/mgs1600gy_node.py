"""MGS1600 Node composable node loader."""
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

from unicodedata import name
from launch.condition import Condition
from launch.conditions import (
    IfCondition,
    UnlessCondition,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from typing import List


def load(
        show: Condition,
        dev: str,
        namespace: str,
        sensor_min=0,
        sensor_max=2000,) -> List[ComposableNodeContainer]:
    """Load composable node container list."""
    viewer = ComposableNode(
        package='mgs1600gy_node',
        plugin='mgs1600gy_node::ShowNode',
        name='mgs1600gy_viewer',
        namespace=namespace,
        parameters=[{
            'name': namespace
        }],
    )
    interface = ComposableNode(
        package='mgs1600gy_node',
        plugin='mgs1600gy_node::Mgs1600gyNode',
        name='mgs1600gy',
        namespace=namespace,
        parameters=[{
            'dev': dev,
            'sensor_min': sensor_min,
            'sensor_max': sensor_max,
        }],
    )

    return [
        ComposableNodeContainer(
            name='mgs1600',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                viewer,
                interface,
            ],
            condition=IfCondition(show),
            output='screen',
        ),
        ComposableNodeContainer(
            name='mgs1600',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                interface,
            ],
            condition=UnlessCondition(show),
            output='screen',
        ),
    ]
