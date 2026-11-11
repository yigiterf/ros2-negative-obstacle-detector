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
    map_yaml = os.path.join(bringup_share, 'maps', 'map.yaml')
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

    # odom -> base_footprint is republished by the odom_to_tf node (from the
    # bridged DiffDrive odometry). map -> odom is published by AMCL once it
    # localizes, so the costmaps and planner track the robot.
    odom_to_tf = Node(
        package='negative_obstacle_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': True}],
        output='screen')

    detector = Node(
        package='negative_obstacle_detector',
        executable='negative_obstacle_detector_node',
        name='negative_obstacle_detector',
        parameters=[detector_params],
        output='screen')

    # Full Nav2 stack: map server, AMCL, NavFn planner, RPP controller,
    # behavior-tree navigator and the lifecycle manager.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'bringup_launch.py'])]),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'True',
            'autostart': 'True',
            'slam': 'False',
        }.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        declare_headless,
        simulation,
        odom_to_tf,
        detector,
        navigation,
        rviz,
    ])
