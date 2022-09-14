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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

from mgs1600gy_bringup import config_loader as cl


def generate_launch_description():
    """Launch rviz display."""
    use_nominal_extrinsics_arg = DeclareLaunchArgument(
        'use_nominal_extrinsics',
        default_value=TextSubstitution(text='True'),
        description='Add sensor link or not')
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            cl.load_robot_description(
                'mgs1600gy.urdf.xacro',
                xacro_options=[
                    ('use_nominal_extrinsics', use_nominal_extrinsics)
                ]
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', cl.load_rviz2('display.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(use_nominal_extrinsics_arg)
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)

    return ld

