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

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from mgs1600gy_bringup import config_loader as cl


def generate_launch_description():
    """Launch sensor."""
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=TextSubstitution(text='info'))
    log_level = LaunchConfiguration('log_level')
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value=TextSubstitution(
            text='/dev/serial/by-id/'
            'usb-Roboteq_Magnetic_Sensor_48F263793238-if00'))
    dev = LaunchConfiguration('dev')
    sensor_min_arg = DeclareLaunchArgument(
        'sensor_min',
        default_value=TextSubstitution(text='-2000'))
    sensor_min = LaunchConfiguration('sensor_min')
    sensor_max_arg = DeclareLaunchArgument(
        'sensor_max',
        default_value=TextSubstitution(text='2000'))
    sensor_max = LaunchConfiguration('sensor_max')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='mgs'))
    namespace = LaunchConfiguration('namespace')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=TextSubstitution(text='True'))
    rviz = LaunchConfiguration('rviz')

    mgs_comp_node = ComposableNode(
        package='mgs1600gy_node',
        name='mgs1600gy',
        plugin='mgs1600gy_node::Mgs1600gyNode',
        parameters=[
            str(get_package_share_path('mgs1600gy_bringup')/'config'/'param.yaml'),
            {
                'dev': dev,
                'sensor_min': sensor_min,
                'sensor_max': sensor_max,
            }])
    tf_comp_node = ComposableNode(
        package='mgs1600gy_node',
        name='imu2tf_node',
        plugin='mgs1600gy_node::Imu2TfNode',
        parameters=[{'frame_id': 'base_link'}])

    mgs_container = ComposableNodeContainer(
        name='mgs_container',
        package='rclcpp_components',
        executable='component_container',
        namespace=namespace,
        composable_node_descriptions=[
            mgs_comp_node,
            tf_comp_node,
        ],
        arguments=['--ros-args', '--log-level', log_level])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            cl.load_robot_description(
                'mgs1600gy.urdf.xacro',
                xacro_options=[('use_nominal_extrinsics', 'True')])],
        condition=IfCondition(rviz))
    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='log',
        condition=IfCondition(rviz),
        arguments=['-d', cl.load_rviz2('mgs1600gy.rviz')])

    ld = LaunchDescription()
    ld.add_action(log_level_arg)
    ld.add_action(dev_arg)
    ld.add_action(sensor_min_arg)
    ld.add_action(sensor_max_arg)
    ld.add_action(rviz_arg)
    ld.add_action(namespace_arg)
    ld.add_action(mgs_container)
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)

    return ld
