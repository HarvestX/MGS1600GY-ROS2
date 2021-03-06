"""Load rviz2 node."""
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


from typing import Callable

from ament_index_python.packages import get_package_share_path

from launch.actions import Shutdown

from launch_ros.actions import Node


def load_node(
    config_filename: str,
    on_exit: Callable = Shutdown
) -> Node:
    """Load node."""
    rviz_config_file = str(
        get_package_share_path('mgs1600gy_bringup') /
        'rviz' /
        config_filename
    )

    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        on_exit=on_exit(),
    )
