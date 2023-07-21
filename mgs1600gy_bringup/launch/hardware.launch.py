# Copyright 2023 HarvestX Inc.
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

from ament_index_python.packages import get_package_share_path
import h6x_bringup_common.config_loader as cl
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    xacro_filepath = get_package_share_path(
        'mgs1600gy_description') / 'urdf' / 'mgs1600gy.urdf.xacro'
    robot_description = cl.load_robot_description(
        xacro_filepath=xacro_filepath,
        xacro_options={
            'dev': '/dev/ttyACM0',
        }.items())

    robot_config_filepath = get_package_share_path('mgs1600gy_bringup') / \
        'config' / 'mgs1600gy_controllers.yaml'

    image_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['image_sensor_broadcaster', '-c', '/controller_manager'])

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '-c', '/controller_manager'])

    ctrl_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description, str(robot_config_filepath)
        ])

    rsp_node = Node(
        namespace='mgs1600gy',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description])

    ld = LaunchDescription()

    ld.add_action(rsp_node)
    ld.add_action(image_broadcaster_spawner)
    ld.add_action(imu_broadcaster_spawner)
    ld.add_action(ctrl_manager_node)

    return ld
