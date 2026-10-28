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

"""Full negative-obstacle stack: simulation, detector, Nav2 costmap, RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for the full stack."""
    det_share = get_package_share_directory('negative_obstacle_detector')
    bringup_share = get_package_share_directory('negative_obstacle_bringup')

    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    detector_params = os.path.join(det_share, 'config', 'detector_params.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'negative_obstacle.rviz')

    headless = LaunchConfiguration('headless')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false', description='Run without Gazebo GUI')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('negative_obstacle_simulation'),
                'launch', 'simulation.launch.py'])]),
        launch_arguments={'headless': headless}.items())

    # Static demo: map -> odom -> base_footprint are fixed identity transforms.
    # (Full dynamic odometry via the DiffDrive bridge requires a
    # robot_localization EKF because Gazebo scopes its TF frame names.)
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])
    odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'])

    detector = Node(
        package='negative_obstacle_detector',
        executable='negative_obstacle_detector_node',
        name='negative_obstacle_detector',
        parameters=[detector_params],
        output='screen')

    costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        parameters=[nav2_params],
        output='screen')

    # The costmap node is a managed (lifecycle) node; drive it to active.
    lifecycle = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 6 && ros2 lifecycle set /costmap configure && '
             'sleep 2 && ros2 lifecycle set /costmap activate'],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        declare_headless,
        simulation,
        map_to_odom,
        odom_to_base,
        detector,
        costmap,
        lifecycle,
        rviz,
    ])
