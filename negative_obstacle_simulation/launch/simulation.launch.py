# Copyright 2026
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

"""Bring up the negative-obstacle simulation: Gazebo world, robot and bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    """Build the launch description for the simulation stack."""
    sim_share = get_package_share_directory('negative_obstacle_simulation')
    desc_share = get_package_share_directory('negative_obstacle_description')

    world = os.path.join(sim_share, 'worlds', 'negative_obstacle_test.sdf')
    bridge_config = os.path.join(sim_share, 'config', 'gz_bridge.yaml')
    urdf = os.path.join(desc_share, 'urdf', 'robot.urdf.xacro')

    headless = LaunchConfiguration('headless')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without the GUI (server only)',
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # Gazebo server and GUI. Headless mode runs the server only.
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world, '-v', '3'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world, '-v', '3'],
        output='screen',
        condition=IfCondition(headless),
    )

    bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config,
    )

    # Spawn the robot from the robot_description topic once the world is ready.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_negative_obstacle_robot',
        parameters=[{
            'world': 'factory_negative_obstacle',
            'topic': 'robot_description',
            'name': 'negative_obstacle_robot',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'R': 0.0,
            'P': 0.0,
            'Y': 0.0,
        }],
        output='screen',
    )

    spawn_delayed = TimerAction(period=3.0, actions=[spawn_robot])

    return LaunchDescription([
        # Force gz-transport onto loopback. Multicast discovery fails on
        # machines without a multicast route, which silently breaks the bridge.
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        declare_headless,
        robot_state_publisher,
        gazebo_gui,
        gazebo_server,
        bridge,
        spawn_delayed,
    ])
