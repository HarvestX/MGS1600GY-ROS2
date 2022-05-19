"""Load robot state publisher node."""
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

from ament_index_python.packages import get_package_share_path

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution
)

from launch_ros.actions import Node


def load_node(filename: str, namespace: str = '') -> Node:
    """Load node."""
    filepath = get_package_share_path(
        'mgs1600gy_description') / 'urdf' / filename
    if 'xacro' in filename:
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                str(filepath)
            ],
        )
    else:
        try:
            with open(str(filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)
    robot_description = {'robot_description': robot_description_content}

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description],
    )

